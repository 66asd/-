#ifndef SCOPE_WAVEFORM_H
#define SCOPE_WAVEFORM_H
#include <stdint.h>
uint16_t Waveform_Stride(float frequency, uint32_t sample_rate, uint16_t capacity);
int Waveform_Render(const uint16_t *samples, uint16_t count, float sample_rate,
                    float frequency, float mean, float vpp, uint8_t *pixels,
                    uint16_t width, uint16_t height);
#endif
