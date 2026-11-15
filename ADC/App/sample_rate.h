#ifndef SAMPLE_RATE_H
#define SAMPLE_RATE_H
#include <stdint.h>
uint32_t SampleRate_Target(float frequency);
uint16_t SampleRate_Stride(uint32_t hardware_rate, float frequency);
typedef struct { uint32_t candidate, since; } SampleRatePolicy;
uint32_t SampleRate_Decide(SampleRatePolicy *p, uint32_t current, float a, float b,
                         int unknown_active, uint32_t now);
#endif
