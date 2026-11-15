#include "measure_vpp.h"
#include <string.h>
#include <math.h>
void Vpp_Reset(VppMeter *m) { memset(m, 0, sizeof(*m)); }
void Vpp_Push(VppMeter *m, uint16_t x)
{
    unsigned b = x >> 4;
    ++m->counts[b];
    m->sums[b] += x;
    ++m->total;
}
static float quantile(const VppMeter *m, uint32_t rank)
{
    uint32_t n = 0u;
    unsigned b;
    for (b = 0u; b < VPP_BINS; ++b) {
        n += m->counts[b];
        if (n > rank) return (float)m->sums[b] / m->counts[b];
    }
    return 0.0f;
}
float Vpp_RobustCounts(const VppMeter *m)
{
    uint32_t trim;
    if (m->total < 2u) return 0.0f;
    trim = m->total / 500u; /* Discard 0.2% of each tail, not the true extrema. */
    return quantile(m, m->total - trim - 1u) - quantile(m, trim);
}
float Vpp_SelectCounts(float robust, float fitted, int sine_valid,
                       float frequency, float sample_rate, uint8_t *used_fit)
{
    float limit;
    *used_fit = 0u;
    if (!sine_valid || robust < 20.0f || frequency <= 0.0f
        || frequency >= sample_rate * 0.45f) return robust;
    /* A sine's sampled extrema can miss by cos(pi*f/fs), not an arbitrary gain. */
    limit = 1.0f / cosf(3.14159265359f * frequency / sample_rate) + 0.03f;
    if (fitted < robust * 0.90f || fitted > robust * limit) return robust;
    *used_fit = 1u;
    return fitted;
}
