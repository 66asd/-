#ifndef SIGNAL_MATH_H
#define SIGNAL_MATH_H
#include <stdint.h>

/* 各占一位，方便同时记录多个问题。例如LOW|FREQ表示幅度小且频率不可信。 */
#define MEAS_FLAG_LOW      1U /* bit0：信号码差太小。 */
#define MEAS_FLAG_CLIP     2U /* bit1：原始码接近0或4095，可能越界/削顶。 */
#define MEAS_FLAG_FREQ     4U /* bit2：频率或相位计算条件不满足。 */
#define MEAS_FLAG_CAPTURE  8U /* bit3：ADC、DMA或内部参考采集失败。 */

typedef struct
{
    float volts_per_code; /* 实测每个ADC码对应的伏特数，不是人为幅度倍率。 */
} MeterCalibration;

/* 数组下标0是S1，1是S2。电压统一存V，发屏幕时才换成mV。 */
typedef struct
{
    float ac_rms_v[2]; /* S1/S2去直流后的交流有效值，单位V。 */
    float frequency_hz[2]; /* 分别由两路实测数据求得的频率，单位Hz。 */
    float phase_deg; /* S2相位减S1相位：负数为S2滞后，正数为S2超前，单位度。 */
    uint32_t flags; /* 全局错误位集合；含两路错误及不能输出相位的原因。 */
    uint32_t channel_flags[2]; /* 每路独立的质量错误；另一通道出错时仍可显示本路有效值。 */
    float vpp_v[2]; /* 最终去噪/拟合后的峰峰值，单位V；发屏幕才转mV。 */
} MeterResult;

/* 内部接口：App传入完整样本、正采样率和本帧实测ADC电压步长，不接收外部任意参数。
 * 只在采集成功后调用。samples低16位是S1，高16位是S2。
 * 相位=S2-S1，负数表示S2滞后；三角波测量的是基波相位。
 */
/* samples：一帧原始打包数组；count：样本对数；fs_hz：每路采样率；cal：物理电压步长来源；result：输出结果。 */
void Signal_Analyze(const uint32_t *samples, uint32_t count, float fs_hz,
                    const MeterCalibration *cal, MeterResult *result);
#endif
