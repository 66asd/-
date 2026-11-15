#ifndef MEASURE_VPP_H
#define MEASURE_VPP_H
#include <stdint.h>
#define VPP_TAIL_SIZE 32u
#define VPP_CYCLE_BINS 32u
typedef struct {
    uint16_t low[VPP_TAIL_SIZE], high[VPP_TAIL_SIZE];
    uint32_t total;
} VppMeter;
typedef struct {
    uint32_t sums[VPP_CYCLE_BINS], phase, step;
    uint16_t counts[VPP_CYCLE_BINS];
    uint16_t low[VPP_CYCLE_BINS], high[VPP_CYCLE_BINS];
    uint8_t bins;
} VppCycleMeter;
void Vpp_Reset(VppMeter *m);
void Vpp_Push(VppMeter *m, uint16_t sample);
float Vpp_RobustCounts(const VppMeter *m);
int VppCycle_Start(VppCycleMeter *m, uint16_t count, float frequency, float sample_rate);
void VppCycle_Push(VppCycleMeter *m, uint16_t sample);
float VppCycle_Result(const VppCycleMeter *m);
float Vpp_SelectCounts(float robust, float fitted, int sine_valid,
                       float frequency, float sample_rate, uint8_t *used_fit);
#endif
