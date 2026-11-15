#include "measure_frequency.h"
#include <string.h>
#include <math.h>

static int add_period(FrequencyMeter *m, uint32_t dt)
{
    uint32_t cycles = 1u;
    if (dt < (2u << 16) || dt > (12000u << 16)) return 0;
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
            if (m->outliers >= 3u && dt >= (2u << 16) && dt <= (12000u << 16)) {
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
        if (!m->candidate && m->armed && m->previous < m->level && x >= m->level) {
            uint32_t frac = ((uint32_t)(m->level - m->previous) << 16)
                          / (uint32_t)(x - m->previous);
            m->candidate_q16 = ((index - 1u) << 16) + frac;
            m->candidate = 1u;
        }
        if (m->candidate) {
            if (x < m->level) m->candidate = 0u;
            else {
                if ((int)x >= (int)m->level + m->hysteresis) {
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
    }
    m->previous = x;
    m->have_previous = 1u;
    return crossed;
}

float Frequency_FinishWindow(FrequencyMeter *m, float sample_rate)
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

void FrequencyRefine_Start(FrequencyRefiner *m,uint16_t count,float frequency,float sample_rate)
{
    float angle=6.28318530718f*frequency/sample_rate;
    memset(m,0,sizeof(*m));
    m->count=count; m->frequency=frequency; m->sample_rate=sample_rate;
    m->cosine=1073741824;
    m->rotate_c=(int32_t)(cosf(angle)*1073741824.0f);
    m->rotate_s=(int32_t)(sinf(angle)*1073741824.0f);
}
void FrequencyRefine_Push(FrequencyRefiner *m,uint16_t sample)
{
    FrequencyProjection *p=&m->half[m->index>=m->count/2u];
    int32_t c=m->cosine>>15,s=m->sine>>15,y=(int32_t)sample-2048;
    int32_t next_c=(int32_t)(((int64_t)m->cosine*m->rotate_c-(int64_t)m->sine*m->rotate_s)>>30);
    m->sine=(int32_t)(((int64_t)m->sine*m->rotate_c+(int64_t)m->cosine*m->rotate_s)>>30);
    m->cosine=next_c;
    p->c+=c; p->s+=s; p->y+=y;
    p->cc+=(int64_t)c*c; p->ss+=(int64_t)s*s; p->cs+=(int64_t)c*s;
    p->yc+=(int64_t)y*c; p->ys+=(int64_t)y*s;
    ++p->count; ++m->index;
}
static int projection(const FrequencyProjection *p,float *cosine,float *sine)
{
    float n=(float)p->count,c,s,y,cc,ss,cs,yc,ys,det;
    if(n<16.0f) return 0;
    c=p->c/32768.0f; s=p->s/32768.0f; y=(float)p->y;
    cc=p->cc/1073741824.0f-c*c/n; ss=p->ss/1073741824.0f-s*s/n;
    cs=p->cs/1073741824.0f-c*s/n;
    yc=p->yc/32768.0f-y*c/n; ys=p->ys/32768.0f-y*s/n;
    det=cc*ss-cs*cs;
    if(det<n*n*0.001f) return 0;
    *cosine=(yc*ss-ys*cs)/det; *sine=(ys*cc-yc*cs)/det;
    return 1;
}
float FrequencyRefine_Result(const FrequencyRefiner *m)
{
    float c[2],s[2],delta,frequency;
    if(!projection(&m->half[0],&c[0],&s[0]) || !projection(&m->half[1],&c[1],&s[1])) return m->frequency;
    /* Separate DC terms reject baseline drift; phase slope refines noisy edge timing. */
    delta=atan2f(c[1]*s[0]-s[1]*c[0],c[1]*c[0]+s[1]*s[0]);
    frequency=m->frequency+delta*m->sample_rate/(3.14159265359f*m->count);
    if(frequency<m->frequency*0.95f || frequency>m->frequency*1.05f) return m->frequency;
    return frequency;
}
