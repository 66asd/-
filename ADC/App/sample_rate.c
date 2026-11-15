#include "sample_rate.h"
#include "scope_config.h"
uint32_t SampleRate_Target(float frequency)
{
    static const uint32_t rates[] = {5000u,10000u,20000u,50000u,100000u,200000u,500000u};
    unsigned i;
    float desired = frequency * SCOPE_TARGET_POINTS;
    for (i = 0u; i < sizeof(rates)/sizeof(rates[0]); ++i)
        if ((float)rates[i] >= desired) return rates[i];
    return SCOPE_SAMPLE_RATE_HZ;
}
uint16_t SampleRate_Stride(uint32_t hardware_rate, float frequency)
{
    uint32_t stride = hardware_rate / SampleRate_Target(frequency);
    return (uint16_t)(stride > 0u ? stride : 1u);
}
uint32_t SampleRate_Decide(SampleRatePolicy *p, uint32_t current, float a, float b,
                         int unknown_active, uint32_t now)
{
    uint32_t target = unknown_active ? SCOPE_SAMPLE_RATE_HZ : SampleRate_Target(a > b ? a : b);
    if (target >= current) { p->candidate = target; p->since = now; return target; }
    /* Downshift with 10% headroom; a boundary must not pin the rate at 500 kS/s. */
    target = SampleRate_Target((a > b ? a : b) / 0.9f);
    if (target >= current) return current;
    if (p->candidate != target) { p->candidate = target; p->since = now; }
    return (uint32_t)(now - p->since) >= SCOPE_RATE_HOLD_MS ? target : current;
}
