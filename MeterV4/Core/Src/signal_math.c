#include "signal_math.h"
#include <math.h>
#include <string.h>

#define PI_F                3.14159265358979323846f
#define MIN_SPAN_CODES      12.0f
#define CHECK_MIN_HZ        350.0f
#define CHECK_MAX_HZ        15000.0f
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


static float trimmed_span(const uint32_t *p, uint32_t count, unsigned ch)
{
    /* shortcut: 2048点各舍弃两端10点；改变帧长时，应一起调整这个数量。
     * 只保存最小/最大各11点，避免复制整帧或占用大块RAM。
     */
    uint16_t low[11], high[11], v;
    uint32_t i, j;
    for (j = 0; j < 11; ++j)
    {
        low[j] = 4095;
        high[j] = 0;
    }
    for (i = 0; i < count; ++i)
    {
        v = (uint16_t)((p[i] >> (ch * 16U)) & 0xFFFU);
        if (v < low[10])
        {
            for (j = 10; j > 0 && v < low[j - 1]; --j)
                low[j] = low[j - 1];
            low[j] = v;
        }
        if (v > high[10])
        {
            for (j = 10; j > 0 && v > high[j - 1]; --j)
                high[j] = high[j - 1];
            high[j] = v;
        }
    }
    /* 第11大减第11小，少量孤立尖峰不会再直接决定Vpp。
     * 不修改样本，也不滤波，因此RMS、频率和相位仍用原来的数据。
     */
    return (float)high[10] - low[10];
}


static float sine_span(const uint32_t *p, uint32_t count, unsigned ch, float fs, float hz)
{
    unsigned pass;
    uint32_t i, n;
    float cs, sn, c, s, next_c, v;
    float sc, ss, cc, zz, cz, sx, xc, xs, det;
    float a = 0, b = 0, dc = 0, amplitude = 0, limit = 0;
    if (hz < CHECK_MIN_HZ || hz > CHECK_MAX_HZ)
        return 0;
    cs = cosf(2 * PI_F * hz / fs);
    sn = sinf(2 * PI_F * hz / fs);

    /* 第一次拟合整帧；第二次排除异常点再拟合，避免少数尖峰拉高幅度。 */
    for (pass = 0; pass < 2; ++pass)
    {
        n = 0;
        sc = ss = cc = zz = cz = sx = xc = xs = 0;
        c = 1;
        s = 0;
        for (i = 0; i < count; ++i)
        {
            v = sample(p, i, ch);
            if (pass == 0 || fabsf(v - dc - a * c - b * s) <= limit)
            {
                ++n;
                sc += c;
                ss += s;
                cc += c * c;
                zz += s * s;
                cz += c * s;
                sx += v;
                xc += v * c;
                xs += v * s;
            }
            next_c = c * cs - s * sn;
            s = s * cs + c * sn;
            c = next_c;
        }
        /* shortcut: 至少90%样本符合正弦模型才使用；其他波形返回截断极值法。 */
        if (n < count - count / 10U)
            return 0;
        cc -= sc * sc / n;
        zz -= ss * ss / n;
        cz -= sc * ss / n;
        xc -= sx * sc / n;
        xs -= sx * ss / n;
        det = cc * zz - cz * cz;
        if (det < 1)
            return 0;
        a = (xc * zz - xs * cz) / det;
        b = (xs * cc - xc * cz) / det;
        dc = (sx - a * sc - b * ss) / n;
        amplitude = sqrtf(a * a + b * b);
        limit = amplitude * 0.06f;
        if (limit < 1)
            limit = 1; /* 保留ADC量化舍入带来的正常波动。 */
    }
    return 2 * amplitude;
}

static void voltage_results(const uint32_t *p, uint32_t count, uint32_t start, uint32_t end,
                            unsigned ch, float fs, const Channel *x,
                            const MeterCalibration *cal, MeterResult *r)
{
    uint32_t i, n = end - start;
    float sum = 0, sq = 0, v, mean, variance, span;
    float step = cal->vdda_v / 4095.0f * cal->gain[ch];

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
    /* 只为Vpp拟合，不改原始样本和其他测量。没有600mV等预设目标值。 */
    span = sine_span(p, count, ch, fs, r->frequency_hz[ch]);
    if (span <= 0)
        span = trimmed_span(p, count, ch);
    r->vpp_v[ch] = span * step;
}

static void phase_difference(const uint32_t *p, uint32_t start, uint32_t end,
                             float fs, const Channel x[2], MeterResult *r)
{
    uint32_t i, n = end - start;
    unsigned ch;
    float omega = 2.0f * PI_F * r->frequency_hz[0] / fs;
    float cs = cosf(omega), sn = sinf(omega);
    float c = 1, s = 0, next_c, v;
    float sc = 0, ss = 0, cc = 0, zz = 0, cz = 0;
    float xc[2] = {0}, xs[2] = {0}, sx[2] = {0};
    float a, b, det, ph[2];

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
    }
    r->phase_deg = (ph[1] - ph[0]) * 180.0f / PI_F;
    if (r->phase_deg > 180)
        r->phase_deg -= 360;
    if (r->phase_deg < -180)
        r->phase_deg += 360;
}

void Signal_Analyze(const uint32_t *p, uint32_t count, float fs,
                    const MeterCalibration *cal, MeterResult *r)
{
    Channel x[2];
    uint32_t start, end;
    unsigned ch;

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

    /* 第三步：只有两路信号质量都合格且同频，才计算相位差。 */
    if (r->flags != 0U)
        return;
    if (fabsf(r->frequency_hz[0] - r->frequency_hz[1]) >
        FREQ_TOLERANCE * r->frequency_hz[0])
    {
        r->flags |= MEAS_FLAG_FREQ;
        return;
    }
    phase_difference(p, start, end, fs, x, r);
}

