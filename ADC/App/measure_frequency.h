#ifndef MEASURE_FREQUENCY_H
#define MEASURE_FREQUENCY_H
#include <stdint.h>
typedef struct {
    uint32_t last_cross_q16, period_sum_q16, periods, candidate_q16, nominal_q16;
    uint32_t last_edge_q16, previous_edge_period;
    uint16_t level, hysteresis, previous;
    uint8_t armed, have_previous, have_cross, enabled, candidate, positive_count, outliers, have_edge;
} FrequencyMeter;
typedef struct {
    int64_t cc,ss,cs,yc,ys;
    int32_t c,s,y;
    uint16_t count;
} FrequencyProjection;
typedef struct {
    FrequencyProjection half[2];
    int32_t cosine,sine,rotate_c,rotate_s;
    uint16_t count,index;
    float frequency,sample_rate;
} FrequencyRefiner;
void FrequencyRefine_Start(FrequencyRefiner *m,uint16_t count,float frequency,float sample_rate);
void FrequencyRefine_Push(FrequencyRefiner *m,uint16_t sample);
float FrequencyRefine_Result(const FrequencyRefiner *m);
void Frequency_Init(FrequencyMeter *m);
void Frequency_SetThreshold(FrequencyMeter *m, uint16_t mean, uint16_t span);
int Frequency_Push(FrequencyMeter *m, uint16_t sample, uint32_t sample_index);
float Frequency_FinishWindow(FrequencyMeter *m, float sample_rate);
#endif
