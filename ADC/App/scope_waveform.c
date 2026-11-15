#include "scope_waveform.h"
#include "scope_config.h"
uint16_t Waveform_Stride(float frequency, uint32_t rate, uint16_t capacity)
{
    float required;
    uint16_t stride;
    if (frequency < 35.0f || capacity < 2u) return 1u;
    required = (float)rate * (SCOPE_DISPLAY_CYCLES + 1u) / frequency
             / (float)(capacity - 1u);
    stride = (uint16_t)required;
    if ((float)stride < required) ++stride;
    return stride > 0u ? stride : 1u;
}
int Waveform_Render(const uint16_t *a, uint16_t count, float rate,
                    float frequency, float mean, float vpp, uint8_t *pixels,
                    uint16_t width, uint16_t height)
{
    uint16_t start = 0u, i;
    uint32_t pos=0u, step;
    int32_t middle, gain_q16;
    float span;
    if (width < 2u || height < 2u || height > 256u || count < 2u) return 0;
    span = count - 2u;
    if (frequency >= 35.0f && vpp >= 20.0f) {
        float wanted = (float)SCOPE_DISPLAY_CYCLES * rate / frequency;
        for (i = 1u; i < count; ++i) {
            if ((float)a[i - 1u] < mean && (float)a[i] >= mean) {
                start=i;
                pos=((uint32_t)(i-1u)<<16)+(uint32_t)((mean-a[i-1u])*65536.0f/(a[i]-a[i-1u]));
                break;
            }
        }
        if ((float)start + wanted < (float)count - 1.0f) span = wanted;
        else pos=0u; /* A short/unlocked frame still displays its real samples. */
    }
    /* Exclude the repeated endpoint: successive four-cycle strips join continuously. */
    step = (uint32_t)(span * 65536.0f / (float)width);
    middle = (int32_t)(mean + 0.5f);
    if (vpp < 20.0f) vpp = 20.0f;
    gain_q16 = (int32_t)((float)(height - 1u) * 0.8f * 65536.0f / vpp);
    for (i = 0u; i < width; ++i) {
        uint32_t k = pos >> 16, frac = pos & 65535u;
        int32_t x = a[k] + ((int32_t)(a[k + 1u] - (int32_t)a[k]) * (int32_t)frac / 65536);
        int32_t y = (int32_t)(height / 2u) + (int32_t)((int64_t)(x - middle) * gain_q16 / 65536);
        if (y < 0) y = 0;
        if (y >= height) y = height - 1u;
        pixels[i] = (uint8_t)y;
        pos += step;
    }
    return 1;
}
