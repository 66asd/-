#include "signal_math.h"
#include <math.h>
#include <string.h>

/* 12位ADC有4096个量化区间；幅值差按每区间电压换算。 */
#define ADC_LEVELS          4096.0f
/* 本模块先在ADC码值域算周期、RMS和峰峰差，最后才按真实电压步长换算成V。
 * p为打包样本数组，count为样本对数，ch=0/1对应S1/S2；fs为每路采样率Hz，hz为信号频率Hz。
 * x保存通道中间统计量，cal提供物理换算单位，r是本帧结果输出。 */
#define PI_F                3.14159265358979323846f
#define MIN_SPAN_CODES      12.0f /* 原始峰谷不足12码时认为信号过小。 */
#define CHECK_MIN_HZ        100.0f /* 频率有效性检查下限；不是输出频率设定。 */
#define CHECK_MAX_HZ        50000.0f /* 频率数值检查上限；不代表整个算法在50kHz仍有保证。 */
#define CROSS_LIMIT_HZ      20000.0f /* 相邻有效穿越至少间隔fs/20000点；实际穿越判定另有限制。 */
#define FREQ_TOLERANCE      0.03f /* 两路测得频率差不超过S1的3%，才按同频处理相位。 */

/* 计算过程用的中间量，单位是ADC码值；最终结果才换算成V。 */
typedef struct
{
    float mean, lo, hi; /* 全帧平均码、最小码、最大码；用于去直流和质量检查。 */
    float first, last; /* 第一次、最后一次有效上升穿越的位置，单位是采样点，可含小数。 */
} Channel;

/* 从一对32位打包数据中取出指定通道的12位原始码；此处不换电压单位。 */
static float sample(const uint32_t *p, uint32_t i, unsigned ch)
{
    /* 同一个32位字的低半部是S1，高半部是S2；每路有效数据只有12位。 */
    return (float)((p[i] >> (ch * 16U)) & 0xFFFU);
}

/* 统计全帧原始均值和峰谷；这些原始峰谷用于质量检查，不直接当最终Vpp。 */
static void statistics(const uint32_t *p, uint32_t count, unsigned ch, Channel *x)
{
    uint32_t i; /* 当前样本下标。 */
    float v; /* 当前通道的原始ADC码。 */
    x->mean = 0;
    x->lo = 4095;
    x->hi = 0;
    for (i = 0; i < count; ++i)
    {
        v = sample(p, i, ch);
        x->mean += v;
        if (v < x->lo)
            x->lo = v;
        if (v > x->hi)
            x->hi = v;
    }
    x->mean /= count;
}

/* 带滞回上升穿越测频：频率=采样率×跨过的周期数/首末穿越点距；失败返回0。 */
static float frequency(const uint32_t *p, uint32_t count, unsigned ch,
                       float fs, Channel *x)
{
    uint32_t i, crossings = 0; /* i为点序号；crossings为已接受的上升穿越次数。 */
    uint8_t armed = 0; /* 先低于负阈值才置1，避免均值附近噪声被重复计为周期。 */
    float a = sample(p, 0, ch) - x->mean; /* 前一点相对均值的偏差，单位码。 */
    float b, t; /* b为当前点偏差；t为本次插值得到的穿越位置。 */
    float hysteresis = (x->hi - x->lo) * 0.1f; /* 峰谷差的10%作为重新允许计数的门槛。 */
    x->first = x->last = 0;

    for (i = 1; i < count; ++i)
    {
        b = sample(p, i, ch) - x->mean;
        /* 先走到均值下方一定距离，才接受下一次上升穿越，避免噪声反复计数。 */
        if (b < -hysteresis)
            armed = 1;
        if (armed && a <= 0 && b > 0)
        {
            /* 在前后两个采样点之间插值，穿越时间不必凑整到某个采样点。 */
            t = (float)(i - 1U) - a / (b - a);
            if (crossings == 0U || t - x->last > fs / CROSS_LIMIT_HZ)
            {
                if (crossings == 0U)
                    x->first = t;
                x->last = t;
                ++crossings;
            }
            armed = 0;
        }
        a = b;
    }
    if (crossings < 3U)
        return 0; /* 穿越太少，不把一个偶然跳变当成稳定周期。 */
    /* N次穿越之间只有N-1个周期；用首末距离平均周期，减少单个穿越误差。 */
    return fs * (crossings - 1U) / (x->last - x->first);
}




/* 为通用Vpp取一个去毛刺样本；window只选择直接取值或三点中值，不是滑动平均长度。 */
static uint32_t vpp_sample(const uint32_t *p, uint32_t count, uint32_t i,
                           unsigned ch, uint32_t window)
{
    uint32_t a, b, c, t; /* a/b/c是前/中/后原始码；t只用于交换，最后b返回三个码的中值。 */
    b = (p[i] >> (ch * 16U)) & 0xFFFU;
    /* 高频每周期点数少，不做中值；首尾缺少邻点时保持原值。 */
    if (window == 1U || i == 0U || i + 1U == count)
        return b;
    a = (p[i - 1U] >> (ch * 16U)) & 0xFFFU;
    c = (p[i + 1U] >> (ch * 16U)) & 0xFFFU;
    if (a > b)
    {
        t = a;
        a = b;
        b = t;
    }
    if (b > c)
        b = c;
    if (a > b)
        b = a;
    return b;
}

/* 通用Vpp：按测得周期将多周期对齐，同相位求平均，再找峰谷；返回值仍是ADC码差。 */
static float filtered_span(const uint32_t *p, uint32_t count, unsigned ch,
                           float fs, float hz, const Channel *x)
{
    /* bin是128个相位位置的序号；cycle是周期序号；cycles是可叠加周期数。
     * i为左侧样本下标；median=3启用三点中值，=1直接取原码。 */
    uint32_t bin, cycle, cycles, i, median;
    /* period为每周期点数；pos为待插值的位置；a/b为左右两点；sum为同相位累加。
     * v为该相位的周期平均码；low/high记录平均后波形的峰谷，不是原始极端噪声点。 */
    float period, pos, sum, a, b, v, low = 4095, high = 0;

    /* 没有可靠周期时不做对齐，保留幅度观察，频率错误仍由原来的标志报告。 */
    if (hz < CHECK_MIN_HZ || hz > CHECK_MAX_HZ)
        return x->hi - x->lo;
    period = fs / hz; /* 500Hz对应400点/周期，10kHz对应20点/周期（当fs=200k）。 */
    cycles = (uint32_t)((x->last - x->first) / period + 0.5f);
    if (cycles == 0U)
        return x->hi - x->lo;
    median = period >= 128.0f ? 3U : 1U; /* 点数足够才做中值，避免高频峰顶被邻点明显压低。 */

    /* 一个周期分成128个相位；先平均各周期的同相位点，再找峰谷。
     * 不把整帧中碰巧最大的噪声点当成峰值，也不需要额外的采样数组。
     */
    for (bin = 0; bin < 128U; ++bin)
    {
        pos = x->first + period * bin / 128.0f;
        sum = 0;
        for (cycle = 0; cycle < cycles; ++cycle)
        {
            i = (uint32_t)pos;
            if (i + 1U >= count)
                break;
            a = (float)vpp_sample(p, count, i, ch, median);
            b = (float)vpp_sample(p, count, i + 1U, ch, median);
            /* 相位通常落在两个样本之间，线性插值避免直接取整造成抖动。 */
            sum += a + (b - a) * (pos - i);
            pos += period;
        }
        if (cycle == 0U)
            continue;
        v = sum / cycle;
        if (v < low)
            low = v;
        if (v > high)
            high = v;
    }
    /* shortcut: 适合稳定周期波；突发信号需要另设测量方式。 */
    return high - low;
}

/* 用窗口内的方差开方计算AC RMS，再算通用峰峰码差；两者乘同一真实每码电压。 */
static void voltage_results(const uint32_t *p, uint32_t count, uint32_t start, uint32_t end,
                            unsigned ch, float fs, const Channel *x,
                            const MeterCalibration *cal, MeterResult *r)
{
    uint32_t i, n = end - start; /* 处理区间[start,end)，n为其中点数；end那一点不参与。 */
    /* v为去掉全帧大致直流后的码值；sum累加v，sq累加v*v。
     * mean为当前窗口残留均值；variance为交流均方；span为去噪后的峰峰码差。 */
    float sum = 0, sq = 0, v, mean, variance, span;
    float step = cal->vdda_v / ADC_LEVELS; /* V4在算法内把本帧VDDA除4096，得到V/码。 */

    for (i = start; i < end; ++i)
    {
        /* 先减掉大部分直流，避免用两个很大的浮点数相减求很小的交流量。 */
        v = sample(p, i, ch) - x->mean;
        sum += v;
        sq += v * v;
    }
    mean = sum / n; /* 当前窗口均值可能与全帧均值略有差别，必须再扣一次。 */
    /* 即平均(v*v)-平均(v)^2，减去残留直流，所以得到交流部分而非含直流的总RMS。 */
    variance = sq / n - mean * mean;
    if (variance < 0)
        variance = 0; /* 浮点舍入可能得到极小负数，不能直接开方。 */

    r->ac_rms_v[ch] = sqrtf(variance) * step;
    span = filtered_span(p, count, ch, fs, r->frequency_hz[ch], x);
    r->vpp_v[ch] = span * step;
}

/* V4两路联合拟合：两路均合格且同频时才进入；共用S1频率与基准cos/sin。
 * 每路模型v≈a*cos+b*sin+直流；正弦判据通过才用拟合幅度更新该路Vpp。 */
static void phase_difference(const uint32_t *p, uint32_t start, uint32_t end,
                             float fs, const Channel x[2], const MeterCalibration *cal, MeterResult *r)
{
    uint32_t i, n = end - start; /* 处理区间[start,end)，n为其中点数；end那一点不参与。 */
    unsigned ch; /* 通道循环序号，0=S1，1=S2。 */
    float omega = 2.0f * PI_F * r->frequency_hz[0] / fs; /* 每个采样点增加多少弧度；两路共用S1频率。 */
    float cs = cosf(omega), sn = sinf(omega); /* 相位增量的cos/sin，用于递推基准，减少三角函数计算。 */
    float c = 1, s = 0, next_c, v; /* 当前cos/sin；next_c暂存下一cos；v为当前去大致直流的码。 */
    /* sc/ss为cos/sin累加；cc/zz为其平方累加；cz为二者乘积累加。 */
    float sc = 0, ss = 0, cc = 0, zz = 0, cz = 0;
    /* 每路各自累计：xc=信号*cos之和，xs=信号*sin之和，sx=信号码值之和。 */
    float xc[2] = {0}, xs[2] = {0}, sx[2] = {0};
    /* a/b为本轮拟合出的余弦/正弦幅度（码），det判断方程能否稳定求解。
     * ph是两路相位（弧度）；power是每路平方和；energy是去直流总能量；fitted是基波拟合能量。 */
    float a, b, det, ph[2], power[2] = {0}, energy, fitted;

    /* 用同一频率拟合两路：信号=A*cos+B*sin+直流。
     * c/s是当前cos/sin；xc/xs是信号分别乘cos/sin后的累加和。
     */
    for (i = start; i < end; ++i)
    {
        sc += c;
        ss += s;
        cc += c * c;
        zz += s * s;
        cz += c * s;
        for (ch = 0; ch < 2; ++ch)
        {
            v = sample(p, i, ch) - x[ch].mean;
            xc[ch] += v * c;
            xs[ch] += v * s;
            sx[ch] += v;
            power[ch] += v * v;
        }
        /* 递推下一点的cos/sin，避免每个点都调用一次三角函数。 */
        next_c = c * cs - s * sn;
        s = s * cs + c * sn;
        c = next_c;
    }

    /* 消除直流项，保留有限采样窗口中正余弦不完全正交的影响。 */
    cc -= sc * sc / n;
    zz -= ss * ss / n;
    cz -= sc * ss / n;
    det = cc * zz - cz * cz;
    if (det < 1.0f)
    {
        r->flags |= MEAS_FLAG_FREQ;
        return; /* 方程无法可靠求解时，不输出一个看似正常的相位。 */
    }
    for (ch = 0; ch < 2; ++ch)
    {
        xc[ch] -= sx[ch] * sc / n;
        xs[ch] -= sx[ch] * ss / n;
        a = (xc[ch] * zz - xs[ch] * cz) / det;
        b = (xs[ch] * cc - xc[ch] * cz) / det;
        ph[ch] = atan2f(-b, a);

        /* 只有正弦拟合解释了99.75%以上交流能量时，才用拟合幅度求Vpp。
         * 三角波、方波保留周期平均的峰谷差，不能套用正弦波公式。
         */
        energy = power[ch] - sx[ch] * sx[ch] / n;
        fitted = a * xc[ch] + b * xs[ch];
        /* 0.9975只判断能否采用正弦模型，不作为输出电压倍率。 */
        if (energy > 0 && fitted >= energy * 0.9975f)
            r->vpp_v[ch] = 2.0f * sqrtf(a * a + b * b) *
                           cal->vdda_v / ADC_LEVELS;
    }
    r->phase_deg = (ph[1] - ph[0]) * 180.0f / PI_F;
    if (r->phase_deg > 180)
        r->phase_deg -= 360;
    if (r->phase_deg < -180)
        r->phase_deg += 360;
}

/* App在DMA采满后调用的总入口：质量/频率 -> RMS/通用Vpp -> 正弦拟合与相位。 */
void Signal_Analyze(const uint32_t *p, uint32_t count, float fs,
                    const MeterCalibration *cal, MeterResult *r)
{
    Channel x[2]; /* 两路各自的平均码、原始峰谷及穿越位置。 */
    uint32_t start, end; /* 尽量选S1完整周期窗口，减少截取半个周期造成的RMS偏差。 */
    unsigned ch; /* 通道循环序号，0=S1，1=S2。 */

    /* 先清掉旧字段，避免信号无效、提前返回时，残留上一帧相位等数据。 */
    memset(r, 0, sizeof(*r));

    /* 第一步：统计幅度、检测质量、分别测量两路频率。 */
    for (ch = 0; ch < 2; ++ch)
    {
        statistics(p, count, ch, &x[ch]);
        if (x[ch].hi - x[ch].lo < MIN_SPAN_CODES)
            r->channel_flags[ch] |= MEAS_FLAG_LOW;
        if (x[ch].lo <= 2 || x[ch].hi >= 4093)
            r->channel_flags[ch] |= MEAS_FLAG_CLIP;

        r->frequency_hz[ch] = frequency(p, count, ch, fs, &x[ch]);
        if (r->frequency_hz[ch] < CHECK_MIN_HZ || r->frequency_hz[ch] > CHECK_MAX_HZ)
            r->channel_flags[ch] |= MEAS_FLAG_FREQ;
        r->flags |= r->channel_flags[ch];
    }

    /* 第二步：两路共用S1首末穿越之间的窗口，尽量按整数周期计算有效值。 */
    start = (uint32_t)ceilf(x[0].first);
    end = (uint32_t)ceilf(x[0].last);
    if (end <= start + 10U || end > count)
    {
        start = 0;
        end = count; /* 没有可靠周期时，仍可用全帧观察幅值，但不承诺相位。 */
    }
    for (ch = 0; ch < 2; ++ch)
        voltage_results(p, count, start, end, ch, fs, &x[ch], cal, r);

    /* 第三步：质量合格且同频时拟合相位，并用正弦拟合改善Vpp。 */
    if (r->flags != 0U)
        return;
    if (fabsf(r->frequency_hz[0] - r->frequency_hz[1]) >
        FREQ_TOLERANCE * r->frequency_hz[0])
    {
        r->flags |= MEAS_FLAG_FREQ;
        return;
    }
    phase_difference(p, start, end, fs, x, cal, r);
}

