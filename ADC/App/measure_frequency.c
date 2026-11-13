#include "measure_frequency.h"
#include <string.h>

static int add_period(FrequencyMeter *m, uint32_t dt)
{
    uint32_t cycles = 1u;
    if (dt < (4u << 16) || dt > (9000u << 16)) return 0;
    if (m->nominal_q16 == 0u) m->nominal_q16 = dt;
    if (dt < m->nominal_q16 * 3u / 5u) return 0;
    if (dt > m->nominal_q16 * 8u / 5u) {
        cycles = (dt + m->nominal_q16 / 2u) / m->nominal_q16;
        if (cycles > 4u) return 0;
    } else {
        m->nominal_q16 = (uint32_t)((int32_t)m->nominal_q16
            + ((int32_t)dt - (int32_t)m->nominal_q16) / 8);
    }
    m->period_sum_q16 += dt;
    m->periods += cycles;
    return 1;
}
static void observe_edge(FrequencyMeter *m, uint32_t t)
{
    if (m->have_edge && m->nominal_q16 != 0u) {
        uint32_t dt = t - m->last_edge_q16;
        if (dt < m->nominal_q16 * 3u / 5u || dt > m->nominal_q16 * 8u / 5u) {
            if (dt >= m->previous_edge_period * 3u / 4u
                && dt <= m->previous_edge_period * 4u / 3u) ++m->outliers;
            else m->outliers = 1u;
            if (m->outliers >= 3u && dt >= (4u << 16) && dt <= (3000u << 16)) {
                /* Persistent adjacent-edge intervals identify a frequency step. */
                m->nominal_q16 = dt;
                m->period_sum_q16 = m->periods = 0u;
                m->have_cross = m->outliers = 0u;
            }
        } else m->outliers = 0u;
        m->previous_edge_period = dt;
    }
    m->last_edge_q16 = t;
    m->have_edge = 1u;
}

void Frequency_Init(FrequencyMeter *m)
{
    memset(m, 0, sizeof(*m));
    m->level = 2048u;
    m->hysteresis = 32u;
    m->enabled = 1u;
}

void Frequency_SetThreshold(FrequencyMeter *m, uint16_t mean, uint16_t span)
{
    int delta = (int)mean - m->level;
    if (delta > (int)m->hysteresis * 2 || delta < -(int)m->hysteresis * 2) {
        m->have_cross = 0u;
        m->armed = 0u;
        m->candidate = 0u;
        m->have_edge = 0u;
        m->nominal_q16 = 0u;
    }
    m->level = mean;
    m->hysteresis = (uint16_t)(span / 16u);
    if (m->hysteresis < 6u) m->hysteresis = 6u;
    m->enabled = (uint8_t)(span >= 20u);
    if (!m->enabled) m->have_cross = m->candidate = m->have_edge = 0u;
}

int Frequency_Push(FrequencyMeter *m, uint16_t x, uint32_t index)
{
    int crossed = 0;
    if (m->enabled && m->have_previous) {
        if ((int)x < (int)m->level - m->hysteresis) m->armed = 1u;
        if (m->candidate) {
            if (x < m->level) m->candidate = 0u;
            else {
                if (m->positive_count < 2u) ++m->positive_count;
                if ((int)x >= (int)m->level + m->hysteresis && m->positive_count >= 2u) {
                    uint32_t t = m->candidate_q16;
                    uint32_t dt = t - m->last_cross_q16;
                    m->candidate = m->armed = 0u;
                    observe_edge(m, t);
                    /* Confirm the excursion, but retain the interpolated crossing time. */
                    if (!m->have_cross || add_period(m, dt)) {
                        m->last_cross_q16 = t;
                        m->have_cross = 1u;
                        crossed = 1;
                    }
                }
            }
        }
        if (!m->candidate && m->armed && m->previous < m->level && x >= m->level) {
            uint32_t frac = ((uint32_t)(m->level - m->previous) << 16)
                          / (uint32_t)(x - m->previous);
            /* Q16 time wraps every 65536 samples; subtraction is intentional. */
            m->candidate_q16 = ((index - 1u) << 16) + frac;
            m->candidate = m->positive_count = 1u;
        }
    }
    m->previous = x;
    m->have_previous = 1u;
    return crossed;
}

float Frequency_FinishWindow(FrequencyMeter *m, uint32_t sample_rate)
{
    float hz = 0.0f;
    if (m->periods >= 2u && m->period_sum_q16 != 0u) {
        hz = (float)sample_rate * 65536.0f * (float)m->periods
           / (float)m->period_sum_q16;
    }
    m->periods = 0u;
    m->period_sum_q16 = 0u;
    if (hz == 0.0f) { m->have_cross = m->have_edge = 0u; m->nominal_q16 = 0u; }
    return hz;
}
