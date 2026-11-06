#ifndef METER_CONFIG_H
#define METER_CONFIG_H

/* 改输出只看这两项；频率单位Hz，波形可选SINE、TRIANGLE、SQUARE。 */
#define METER_DDS_HZ              500U
#define METER_DDS_WAVE            AD9833_WAVE_SINE

/* 一帧采2048对样本；250ms启动一帧，不是每250ms采一个点。 */
#define METER_SAMPLE_COUNT        2048U
#define METER_UPDATE_PERIOD_MS    250U
#define METER_CAPTURE_TIMEOUT_MS  100U

/* ADC码值换算为实际输入电压。这里用实测供电和校准值，不凭预期读数放大。 */
#define METER_VDDA_V              3.300f
/* 按用户给出的500Hz、610mV参考做单点幅值校准，两个通道分别计算。
 * 系数固定，不按每帧读数反推；换板、换前端后需重新校准。
 * RMS和Vpp共用同一系数，保持电压尺度一致；设回1.0f可单看算法结果。
 */
#define METER_GAIN_S1             0.98590665f
#define METER_GAIN_S2             0.98731986f

#endif
