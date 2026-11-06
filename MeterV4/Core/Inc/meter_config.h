#ifndef METER_CONFIG_H
#define METER_CONFIG_H

/* 改输出只看这两项；频率单位Hz，波形可选SINE、TRIANGLE、SQUARE。 */
#define METER_DDS_HZ              500U
#define METER_DDS_WAVE            AD9833_WAVE_SINE

/* 一帧采2048对样本；250ms启动一帧，不是每250ms采一个点。 */
#define METER_SAMPLE_COUNT        2048U
#define METER_UPDATE_PERIOD_MS    250U
#define METER_CAPTURE_TIMEOUT_MS  100U

/* STM32F103内部参考的典型电压，来自芯片规格，不是通道幅值修正系数。
 * 内部参考有出厂容差，因此估算VDDA不能替代精密电压基准。
 */
#define METER_VREFINT_TYP_V       1.200f

#endif
