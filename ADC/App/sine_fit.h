#ifndef SINE_FIT_H
#define SINE_FIT_H
#include <stdint.h>
typedef struct {
    const uint16_t *samples;
    uint32_t phase, step;
    uint16_t count, index, used, rejected;
    uint8_t pass, done, valid;
    int64_t cc, ss, cs, yc, ys, yy;
    int32_t c, s, y;
    float dc, cosine, sine, amplitude, residual_ratio;
    int32_t dc_q8, cosine_q8, sine_q8, reject_counts;
} SineFit;
void SineFit_Start(SineFit *f, const uint16_t *samples, uint16_t count,
                   float sample_rate, float frequency);
int SineFit_Step(SineFit *f, uint16_t budget);
#endif
