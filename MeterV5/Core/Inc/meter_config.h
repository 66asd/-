#ifndef METER_CONFIG_H
#define METER_CONFIG_H

/* 改输出只看这两项；频率单位Hz，波形可选SINE、TRIANGLE、SQUARE。 */
#define METER_DDS_HZ              10000U /* DDS期望频率；当前保留用户改成的10kHz。 */
#define METER_DDS_WAVE            AD9833_WAVE_SINE

/* 一帧采4096对样本；250ms启动一帧，不是每250ms采一个点。 */
#define METER_SAMPLE_COUNT        4096U /* 一帧4096对S1/S2；200kS/s时采样历时20.48ms。 */
#define METER_UPDATE_PERIOD_MS    250U /* 两次开始测量至少隔250ms，忙时不重入；不是采样间隔。 */
#define METER_CAPTURE_TIMEOUT_MS  100U /* 只给主采样等待用的超时；不是RMS或Vpp滤波窗口。 */

/* V5名义启动量程固定3.3V；有效帧的每码电压由真实VREF测量给出。 */
#define METER_VDDA_V              3.3f

/* 本板真实标定记录保存在capture.c；不使用典型1.2V或目标Vpp反推值。 */

#endif
