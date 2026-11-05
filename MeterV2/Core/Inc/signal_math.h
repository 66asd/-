#ifndef SIGNAL_MATH_H
#define SIGNAL_MATH_H
#include <stdint.h>
#define MEAS_FLAG_LOW 1U
#define MEAS_FLAG_CLIP 2U
#define MEAS_FLAG_FREQ 4U
#define MEAS_FLAG_TIMEOUT 8U
#define MEAS_FLAG_CAL 16U
typedef struct
{
    float vdda_v, gain[2], offset_v[2];
} MeterCalibration;
typedef struct
{
    float ac_rms_v[2], dc_v[2], frequency_hz[2], phase_deg;
    uint32_t flags, elapsed_ms, channel_flags[2];
    float min_v[2], max_v[2], total_rms_v[2], vpp_v[2];
} MeterResult;
/* Volts internally. AC excludes DC; total RMS includes DC.
 * Phase = phase(S2)-phase(S1), negative means S2 lags; triangle uses fundamental.
 * Input is packed ADC1/ADC2, sampled simultaneously at fs_hz. No HAL dependency. */
void Signal_Analyze(const uint32_t *samples, uint32_t count, float fs_hz,
                    const MeterCalibration *cal, MeterResult *result);
#endif
