#include "measure_rms.h"
#include <math.h>
#include <string.h>
void Rms_Reset(RmsMeter *m) { memset(m, 0, sizeof(*m)); }
void Rms_Push(RmsMeter *m, uint16_t x)
{
    m->all.sum += x;
    m->all.squares += (uint32_t)x * x;
    ++m->all.count;
}
void Rms_MarkCycle(RmsMeter *m)
{
    if (m->marks == 0u) m->first = m->all;
    m->last = m->all;
    ++m->marks;
}
RmsResult Rms_Result(const RmsMeter *m)
{
    RmsMoments a = m->all;
    RmsResult r = {0.0f, 0.0f, 0.0f};
    uint64_t dc_energy;
    if (m->marks >= 2u && m->last.count - m->first.count >= 10u) {
        a.count = m->last.count - m->first.count;
        a.sum = m->last.sum - m->first.sum;
        a.squares = m->last.squares - m->first.squares;
    }
    if (a.count == 0u) return r;
    r.mean_counts = (float)a.sum / (float)a.count;
    dc_energy = a.sum * a.sum / a.count;
    /* Remove DC in integer arithmetic before conversion to float. */
    r.ac_counts = sqrtf((float)(a.squares - dc_energy) / (float)a.count);
    r.total_counts = sqrtf((float)a.squares / (float)a.count);
    return r;
}
