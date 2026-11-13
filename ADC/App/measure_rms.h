#ifndef MEASURE_RMS_H
#define MEASURE_RMS_H
#include <stdint.h>
typedef struct { uint64_t sum, squares; uint32_t count; } RmsMoments;
typedef struct {
    RmsMoments all, first, last;
    uint32_t marks;
} RmsMeter;
typedef struct { float mean_counts, ac_counts, total_counts; } RmsResult;
void Rms_Reset(RmsMeter *m);
void Rms_Push(RmsMeter *m, uint16_t x);
void Rms_MarkCycle(RmsMeter *m);
RmsResult Rms_Result(const RmsMeter *m);
#endif
