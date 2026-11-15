#include "measure_vpp.h"
#include <string.h>
#include <math.h>
void Vpp_Reset(VppMeter *m) { memset(m, 0, sizeof(*m)); }
void Vpp_Push(VppMeter *m, uint16_t x)
{
    unsigned kept=m->total<VPP_TAIL_SIZE?m->total+1u:VPP_TAIL_SIZE;
    unsigned i;
    if(m->total<VPP_TAIL_SIZE || x<m->low[kept-1u]) {
        i=kept-1u;
        while(i && x<m->low[i-1u]) { m->low[i]=m->low[i-1u]; --i; }
        m->low[i]=x;
    }
    if(m->total<VPP_TAIL_SIZE || x>m->high[kept-1u]) {
        i=kept-1u;
        while(i && x>m->high[i-1u]) { m->high[i]=m->high[i-1u]; --i; }
        m->high[i]=x;
    }
    ++m->total;
}
float Vpp_RobustCounts(const VppMeter *m)
{
    uint32_t trim;
    if (m->total < 2u) return 0.0f;
    trim = m->total / 500u; /* Discard 0.2% of each tail, not the true extrema. */
    /* The application uses 1024 points; exact ranks fit through 15999 points. */
    if(trim>=VPP_TAIL_SIZE) trim=VPP_TAIL_SIZE-1u;
    return (float)m->high[trim]-(float)m->low[trim];
}
int VppCycle_Start(VppCycleMeter *m, uint16_t count, float frequency, float sample_rate)
{
    unsigned bins;
    memset(m,0,sizeof(*m));
    /* shortcut: require eight cycles and >=8 points/cycle for nonparametric averaging. */
    if(frequency<=0.0f || frequency*8.0f>sample_rate ||
       (float)count*frequency<sample_rate*8.0f) return 0;
    bins=(unsigned)(sample_rate/frequency+0.5f);
    if(bins>VPP_CYCLE_BINS) bins=VPP_CYCLE_BINS;
    m->bins=(uint8_t)bins;
    m->step=(uint32_t)(frequency/sample_rate*4294967296.0f);
    return 1;
}
void VppCycle_Push(VppCycleMeter *m, uint16_t x)
{
    unsigned b=(unsigned)(((uint64_t)m->phase*m->bins+0x80000000u)>>32);
    if(b==m->bins) b=0u;
    if(!m->counts[b]) m->low[b]=m->high[b]=x;
    if(x<m->low[b]) m->low[b]=x;
    if(x>m->high[b]) m->high[b]=x;
    m->sums[b]+=x; ++m->counts[b]; m->phase+=m->step;
}
float VppCycle_Result(const VppCycleMeter *m)
{
    float low=4095.0f,high=0.0f;
    unsigned b;
    if(!m->bins) return 0.0f;
    for(b=0u;b<m->bins;++b) {
        float mean;
        if(m->counts[b]<4u) return 0.0f;
        /* Drop one extreme at each phase, so isolated raw spikes cannot set VPP. */
        mean=m->counts[b]>=6u?(float)(m->sums[b]-m->low[b]-m->high[b])/(m->counts[b]-2u):
                             (float)m->sums[b]/m->counts[b];
        if(mean<low) low=mean;
        if(mean>high) high=mean;
    }
    return high-low;
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
