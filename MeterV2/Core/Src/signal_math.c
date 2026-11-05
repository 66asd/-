#include "signal_math.h"
#include <math.h>
#include <string.h>
#define PI_F 3.14159265358979323846f
static float value(const uint32_t *p, uint32_t i, unsigned ch)
{
    return (float)((p[i] >> (ch * 16U)) & 0xFFFU);
}
/* Mean-level rising crossings with hysteresis, linear time interpolation. */
static float crossings(const uint32_t *p, unsigned ch, uint32_t count, float fs, float mean,
                       float hysteresis, float *first, float *last)
{
    uint32_t i, n_cross = 0;
    int armed = 0;
    float a = value(p, 0, ch) - mean, b, t;
    *first = *last = 0.0f;
    for (i = 1; i < count; ++i)
    {
        b = value(p, i, ch) - mean;
        if (b < -hysteresis)
            armed = 1;
        if (armed && a <= 0.0f && b > 0.0f)
        {
            t = (float)(i - 1U) - a / (b - a);
            if (n_cross == 0U || t - *last > fs / 20000.0f)
            {
                if (n_cross == 0U)
                    *first = t;
                *last = t;
                ++n_cross;
            }
            armed = 0;
        }
        a = b;
    }
    return (n_cross >= 3U) ? fs * (float)(n_cross - 1U) / (*last - *first) : 0.0f;
}
void Signal_Analyze(const uint32_t *p, uint32_t count, float fs, const MeterCalibration *cal,
                    MeterResult *r)
{
    uint32_t i, start, end, n;
    unsigned ch;
    float mean[2] = {0}, lo[2] = {4095, 4095}, hi[2] = {0};
    float first[2], last[2], sum, sq, v, variance;
    float c, s, cs, sn, nc, omega;
    float xc[2] = {0}, xs[2] = {0}, sx[2] = {0};
    float sc = 0, ss = 0, cc = 0, zz = 0, cz = 0, det, a[2], b[2], ph[2];
    if (!r)
        return;
    memset(r, 0, sizeof(*r));
    if (!p || !cal || count < 16U || !(fs >= 1000.0f && fs <= 2000000.0f) ||
        !(cal->vdda_v >= 2.4f && cal->vdda_v <= 3.6f) ||
        !(cal->gain[0] > 0 && cal->gain[0] <= 100.0f) ||
        !(cal->gain[1] > 0 && cal->gain[1] <= 100.0f) ||
        !(cal->offset_v[0] >= -100.0f && cal->offset_v[0] <= 100.0f) ||
        !(cal->offset_v[1] >= -100.0f && cal->offset_v[1] <= 100.0f))
    {
        r->flags = MEAS_FLAG_CAL;
        return;
    }
    for (i = 0; i < count; ++i)
        for (ch = 0; ch < 2; ++ch)
        {
            v = value(p, i, ch);
            mean[ch] += v;
            if (v < lo[ch])
                lo[ch] = v;
            if (v > hi[ch])
                hi[ch] = v;
        }
    for (ch = 0; ch < 2; ++ch)
    {
        uint32_t cf = 0;
        mean[ch] /= (float)count;
        r->min_v[ch] = lo[ch] * cal->vdda_v / 4095.0f * cal->gain[ch] + cal->offset_v[ch];
        r->max_v[ch] = hi[ch] * cal->vdda_v / 4095.0f * cal->gain[ch] + cal->offset_v[ch];
        r->vpp_v[ch] = r->max_v[ch] - r->min_v[ch];
        r->dc_v[ch] = mean[ch] * cal->vdda_v / 4095.0f * cal->gain[ch] + cal->offset_v[ch];
        if (hi[ch] - lo[ch] < 12.0f)
            cf |= MEAS_FLAG_LOW;
        if (lo[ch] <= 2.0f || hi[ch] >= 4093.0f)
            cf |= MEAS_FLAG_CLIP;
        r->frequency_hz[ch] =
            crossings(p, ch, count, fs, mean[ch], (hi[ch] - lo[ch]) * 0.1f, &first[ch], &last[ch]);
        if (r->frequency_hz[ch] < 350.0f || r->frequency_hz[ch] > 15000.0f)
            cf |= MEAS_FLAG_FREQ;
        r->channel_flags[ch] = cf;
        r->flags |= cf;
    }
    /* Use nearly integral S1 periods for AC RMS; do not assume sine/triangle. */
    start = (uint32_t)ceilf(first[0]);
    end = (uint32_t)ceilf(last[0]);
    if (end <= start + 10U || end > count)
    {
        start = 0;
        end = count;
    }
    n = end - start;
    for (ch = 0; ch < 2; ++ch)
    {
        sum = 0;
        sq = 0;
        for (i = start; i < end; ++i)
        {
            v = value(p, i, ch) - mean[ch];
            sum += v;
            sq += v * v;
        }
        variance = sq / (float)n - (sum / (float)n) * (sum / (float)n);
        r->ac_rms_v[ch] =
            sqrtf(variance > 0 ? variance : 0) * cal->vdda_v / 4095.0f * cal->gain[ch];
        r->dc_v[ch] =
            (mean[ch] + sum / (float)n) * cal->vdda_v / 4095.0f * cal->gain[ch] + cal->offset_v[ch];
        r->total_rms_v[ch] = sqrtf(r->dc_v[ch] * r->dc_v[ch] + r->ac_rms_v[ch] * r->ac_rms_v[ch]);
    }
    if (r->flags & (MEAS_FLAG_LOW | MEAS_FLAG_FREQ))
        return;
    if (fabsf(r->frequency_hz[0] - r->frequency_hz[1]) > 0.03f * r->frequency_hz[0])
    {
        r->flags |= MEAS_FLAG_FREQ;
        return;
    }
    /* Fit x=A*cos(wt)+B*sin(wt)+DC using shared frequency, including the
       finite-window Gram matrix. Phase is the fundamental phase for triangles. */
    omega = 2.0f * PI_F * r->frequency_hz[0] / fs;
    cs = cosf(omega);
    sn = sinf(omega);
    c = 1;
    s = 0;
    for (i = start; i < end; ++i)
    {
        sc += c;
        ss += s;
        cc += c * c;
        zz += s * s;
        cz += c * s;
        for (ch = 0; ch < 2; ++ch)
        {
            v = value(p, i, ch) - mean[ch];
            xc[ch] += v * c;
            xs[ch] += v * s;
            sx[ch] += v;
        }
        nc = c * cs - s * sn;
        s = s * cs + c * sn;
        c = nc;
    }
    cc -= sc * sc / (float)n;
    zz -= ss * ss / (float)n;
    cz -= sc * ss / (float)n;
    det = cc * zz - cz * cz;
    if (det < 1.0f)
    {
        r->flags |= MEAS_FLAG_FREQ;
        return;
    }
    for (ch = 0; ch < 2; ++ch)
    {
        xc[ch] -= sx[ch] * sc / (float)n;
        xs[ch] -= sx[ch] * ss / (float)n;
        a[ch] = (xc[ch] * zz - xs[ch] * cz) / det;
        b[ch] = (xs[ch] * cc - xc[ch] * cz) / det;
        ph[ch] = atan2f(-b[ch], a[ch]);
    }
    r->phase_deg = (ph[1] - ph[0]) * 180.0f / PI_F;
    if (r->phase_deg > 180)
        r->phase_deg -= 360;
    if (r->phase_deg < -180)
        r->phase_deg += 360;
}
