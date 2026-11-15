#ifndef MEASURE_VPP_H
#define MEASURE_VPP_H
#include <stdint.h>
#define VPP_BINS 256u
typedef struct {
    uint16_t counts[VPP_BINS];
    uint32_t sums[VPP_BINS], total;
} VppMeter;
void Vpp_Reset(VppMeter *m);
void Vpp_Push(VppMeter *m, uint16_t sample);
float Vpp_RobustCounts(const VppMeter *m);
float Vpp_SelectCounts(float robust, float fitted, int sine_valid,
                       float frequency, float sample_rate, uint8_t *used_fit);
#endif
