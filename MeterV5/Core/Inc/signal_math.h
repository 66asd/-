#ifndef SIGNAL_MATH_H
#define SIGNAL_MATH_H
#include <stdint.h>

/* 各占一位，方便同时记录多个问题。例如LOW|FREQ表示幅度小且频率不可信。 */
#define MEAS_FLAG_LOW      1U
#define MEAS_FLAG_CLIP     2U
#define MEAS_FLAG_FREQ     4U
#define MEAS_FLAG_CAPTURE  8U

typedef struct
{
    float vdda_v;
} MeterCalibration;

/* 数组下标0是S1，1是S2。电压统一存V，发屏幕时才换成mV。 */
typedef struct
{
    float ac_rms_v[2];
    float frequency_hz[2];
    float phase_deg;
    uint32_t flags;
    uint32_t channel_flags[2];
    float vpp_v[2];
} MeterResult;

/* 内部接口：App传入完整样本、正采样率和固定校准参数，不接收外部任意参数。
 * 只在采集成功后调用。samples低16位是S1，高16位是S2。
 * 相位=S2-S1，负数表示S2滞后；三角波测量的是基波相位。
 */
void Signal_Analyze(const uint32_t *samples, uint32_t count, float fs_hz,
                    const MeterCalibration *cal, MeterResult *result);
#endif
