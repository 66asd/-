#ifndef MEASURE_FREQUENCY_H
#define MEASURE_FREQUENCY_H
#include <stdint.h>
typedef struct {
    uint32_t last_cross_q16, period_sum_q16, periods, candidate_q16, nominal_q16;
    uint32_t last_edge_q16, previous_edge_period;
    uint16_t level, hysteresis, previous;
    uint8_t armed, have_previous, have_cross, enabled, candidate, positive_count, outliers, have_edge;
} FrequencyMeter;
void Frequency_Init(FrequencyMeter *m);
void Frequency_SetThreshold(FrequencyMeter *m, uint16_t mean, uint16_t span);
int Frequency_Push(FrequencyMeter *m, uint16_t sample, uint32_t sample_index);
float Frequency_FinishWindow(FrequencyMeter *m, float sample_rate);
#endif
