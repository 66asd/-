#include "signal_math.h"
#include <math.h>
#include <string.h>

/* 12位ADC有4096个量化区间；幅值差按每区间电压换算。 */
#define ADC_LEVELS          4096.0f
#define PI_F                3.14159265358979323846f
#define MIN_SPAN_CODES      12.0f
#define CHECK_MIN_HZ        100.0f
#define CHECK_MAX_HZ        50000.0f
#define CROSS_LIMIT_HZ      20000.0f
#define FREQ_TOLERANCE      0.03f

/* 计算过程用的中间量，单位是ADC码值；最终结果才换算成V。 */
typedef struct
{
    float mean, lo, hi;
    float first, last;
} Channel;

static float sample(const uint32_t *p, uint32_t i, unsigned ch)
{
    /* 同一个32位字的低半部是S1，高半部是S2；每路有效数据只有12位。 */
    return (float)((p[i] >> (ch * 16U)) & 0xFFFU);
}

static void statistics(const uint32_t *p, uint32_t count, unsigned ch, Channel *x)
{
    uint32_t i;
    float v;
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

static float frequency(const uint32_t *p, uint32_t count, unsigned ch,
                       float fs, Channel *x)
{
    uint32_t i, crossings = 0;
    uint8_t armed = 0;
    float a = sample(p, 0, ch) - x->mean;
    float b, t;
    float hysteresis = (x->hi - x->lo) * 0.1f;
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
    return fs * (crossings - 1U) / (x->last - x->first);
}




static uint32_t vpp_sample(const uint32_t *p, uint32_t count, uint32_t i,
                           unsigned ch, uint32_t window)
{
    uint32_t a, b, c, t;
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

static float filtered_span(const uint32_t *p, uint32_t count, unsigned ch,
                           float fs, float hz, const Channel *x)
{
    uint32_t bin, cycle, cycles, i, median;
    float period, pos, sum, a, b, v, low = 4095, high = 0;

    /* 没有可靠周期时不做对齐，保留幅度观察，频率错误仍由原来的标志报告。 */
    if (hz < CHECK_MIN_HZ || hz > CHECK_MAX_HZ)
        return x->hi - x->lo;
    period = fs / hz;
    cycles = (uint32_t)((x->last - x->first) / period + 0.5f);
    if (cycles == 0U)
        return x->hi - x->lo;
    median = period >= 128.0f ? 3U : 1U;

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

static void voltage_results(const uint32_t *p, uint32_t count, uint32_t start, uint32_t end,
                            unsigned ch, float fs, const Channel *x,
                            const MeterCalibration *cal, MeterResult *r)
{
    uint32_t i, n = end - start;
    float sum = 0, sq = 0, v, mean, variance, span;
    float step = cal->vdda_v / ADC_LEVELS;

    for (i = start; i < end; ++i)
    {
        /* 先减掉大部分直流，避免用两个很大的浮点数相减求很小的交流量。 */
        v = sample(p, i, ch) - x->mean;
        sum += v;
        sq += v * v;
    }
    mean = sum / n;
    variance = sq / n - mean * mean;
    if (variance < 0)
        variance = 0; /* 浮点舍入可能得到极小负数，不能直接开方。 */

    r->ac_rms_v[ch] = sqrtf(variance) * step;
    span = filtered_span(p, count, ch, fs, r->frequency_hz[ch], x);
    r->vpp_v[ch] = span * step;
}

static void fit_channel(const uint32_t *p, uint32_t start, uint32_t end,
                        unsigned ch, float hz, float fs, const Channel *x,
                        const MeterCalibration *cal, MeterResult *r, float *phase)
{
    uint32_t i, n = end - start;
    float omega = 2.0f * PI_F * hz / fs;
    float cs = cosf(omega), sn = sinf(omega);
    float c = 1, s = 0, next_c, v;
    float sc = 0, ss = 0, cc = 0, zz = 0, cz = 0;
    float xc = 0, xs = 0, sx = 0, power = 0;
    float a, b, det, energy, fitted;

    /* 每路独立拟合，另一通道断线或不同频不应改变本路Vpp算法。 */
    for (i = start; i < end; ++i)
    {
        sc += c;
        ss += s;
        cc += c * c;
        zz += s * s;
        cz += c * s;
        v = sample(p, i, ch) - x->mean;
        xc += v * c;
        xs += v * s;
        sx += v;
        power += v * v;
        next_c = c * cs - s * sn;
        s = s * cs + c * sn;
        c = next_c;
    }
    cc -= sc * sc / n;
    zz -= ss * ss / n;
    cz -= sc * ss / n;
    xc -= sx * sc / n;
    xs -= sx * ss / n;
    det = cc * zz - cz * cz;
    if (det < 1.0f)
    {
        r->flags |= MEAS_FLAG_FREQ;
        return;
    }
    a = (xc * zz - xs * cz) / det;
    b = (xs * cc - xc * cz) / det;
    *phase = atan2f(-b, a);
    energy = power - sx * sx / n;
    fitted = a * xc + b * xs;
    /* 三角波、方波不能套用正弦幅度，继续使用周期平均峰谷差。 */
    if (energy > 0 && fitted >= energy * 0.9975f)
        r->vpp_v[ch] = 2.0f * sqrtf(a * a + b * b) *
                       cal->vdda_v / ADC_LEVELS;
}

void Signal_Analyze(const uint32_t *p, uint32_t count, float fs,
                    const MeterCalibration *cal, MeterResult *r)
{
    Channel x[2];
    uint32_t start, end;
    unsigned ch;
    uint8_t same_frequency;
    float phase[2] = {0}, hz;

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

    /* 同频的两路使用同一拟合频率和窗口，保持相位的共同时间基准。 */
    same_frequency = r->flags == 0U &&
        fabsf(r->frequency_hz[0] - r->frequency_hz[1]) <=
        FREQ_TOLERANCE * r->frequency_hz[0];
    for (ch = 0; ch < 2; ++ch)
    {
        if (r->channel_flags[ch] == 0U)
        {
            hz = same_frequency ? r->frequency_hz[0] : r->frequency_hz[ch];
            fit_channel(p, start, end, ch, hz, fs, &x[ch], cal, r, &phase[ch]);
        }
    }
    if (r->flags != 0U)
        return;
    if (!same_frequency)
    {
        r->flags |= MEAS_FLAG_FREQ;
        return;
    }
    r->phase_deg = (phase[1] - phase[0]) * 180.0f / PI_F;
    if (r->phase_deg > 180)
        r->phase_deg -= 360;
    if (r->phase_deg < -180)
        r->phase_deg += 360;
}
