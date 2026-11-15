#include "sine_fit.h"
#include <math.h>
#include <string.h>
#include "sine_table.inc"

static int32_t sin_q15(uint32_t phase)
{
    uint32_t pos = phase & 0x3fffffffu;
    uint32_t i, frac;
    int32_t value;
    if ((phase & 0x40000000u) != 0u) pos = 0x40000000u - pos;
    i = pos >> 23;
    if (i >= 128u) value = 32767;
    else {
        frac = (pos >> 8) & 32767u;
        value = quarter_sine[i]
              + ((quarter_sine[i + 1u] - quarter_sine[i]) * (int32_t)frac >> 15);
    }
    return (phase & 0x80000000u) != 0u ? -value : value;
}

static void clear_sums(SineFit *f)
{
    f->phase = 0u;
    f->index = f->used = f->rejected = 0u;
    f->cc = f->ss = f->cs = f->yc = f->ys = f->yy = 0;
    f->c = f->s = f->y = 0;
}

void SineFit_Start(SineFit *f, const uint16_t *samples, uint16_t count,
                   float sample_rate, float frequency)
{
    memset(f, 0, sizeof(*f));
    f->samples = samples;
    f->count = count;
    f->done = 1u;
    if (count < 16u || frequency <= 0.0f || frequency >= sample_rate * 0.45f
        || (float)count * frequency < sample_rate * 1.5f) return;
    f->step = (uint32_t)(frequency / sample_rate * 4294967296.0f);
    f->done = 0u;
}

static int solve(SineFit *f)
{
    float n = (float)f->used, sc, ss, sy, cc, s2, cs, yc, ys, det, energy, error;
    if (f->used < 16u) return 0;
    sc = (float)f->c / 32768.0f;
    ss = (float)f->s / 32768.0f;
    sy = (float)f->y;
    cc = (float)f->cc / 1073741824.0f - sc * sc / n;
    s2 = (float)f->ss / 1073741824.0f - ss * ss / n;
    cs = (float)f->cs / 1073741824.0f - sc * ss / n;
    yc = (float)f->yc / 32768.0f - sy * sc / n;
    ys = (float)f->ys / 32768.0f - sy * ss / n;
    det = cc * s2 - cs * cs;
    if (det < n * n * 0.001f) return 0;
    f->cosine = (yc * s2 - ys * cs) / det;
    f->sine = (ys * cc - yc * cs) / det;
    f->dc = 2048.0f + (sy - f->cosine * sc - f->sine * ss) / n;
    f->amplitude = sqrtf(f->cosine * f->cosine + f->sine * f->sine);
    energy = (float)f->yy - sy * sy / n;
    error = energy - f->cosine * yc - f->sine * ys;
    if (error < 0.0f) error = 0.0f;
    f->residual_ratio = energy > 0.0f ? error / energy : 1.0f;
    /* Real ADC captures have ~0.39% residual energy; keep triangle/square rejection. */
    return f->amplitude >= 10.0f && f->amplitude < 2100.0f
        && error <= energy * 0.005f + n * 2.0f;
}

int SineFit_Step(SineFit *f, uint16_t budget)
{
    if (f->done) return 1;
    while (budget-- != 0u && f->index < f->count) {
        int32_t c = sin_q15(f->phase + 0x40000000u);
        int32_t s = sin_q15(f->phase);
        int32_t y = (int32_t)f->samples[f->index] - 2048;
        int include = 1;
        if (f->pass != 0u) {
            int32_t predicted = f->dc_q8
                + (int32_t)(((int64_t)f->cosine_q8 * c
                            + (int64_t)f->sine_q8 * s) >> 15);
            int32_t residual = ((y + 2048) * 256 - predicted) / 256;
            include = residual <= f->reject_counts && residual >= -f->reject_counts;
        }
        if (include) {
            f->c += c; f->s += s; f->y += y;
            f->cc += (int64_t)c * c;
            f->ss += (int64_t)s * s;
            f->cs += (int64_t)c * s;
            f->yc += (int64_t)y * c;
            f->ys += (int64_t)y * s;
            f->yy += (int64_t)y * y;
            ++f->used;
        } else ++f->rejected;
        f->phase += f->step;
        ++f->index;
    }
    if (f->index == f->count) {
        int good = solve(f);
        if (f->pass == 0u && f->amplitude >= 10.0f) {
            f->dc_q8 = (int32_t)(f->dc * 256.0f);
            f->cosine_q8 = (int32_t)(f->cosine * 256.0f);
            f->sine_q8 = (int32_t)(f->sine * 256.0f);
            f->reject_counts = (int32_t)(f->amplitude * 0.15f) + 8;
            f->pass = 1u;
            clear_sums(f);
        } else {
            f->valid = (uint8_t)(good && (uint32_t)f->rejected * 50u <= f->count);
            f->done = 1u;
        }
    }
    return f->done;
}
