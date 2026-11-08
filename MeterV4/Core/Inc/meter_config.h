#ifndef METER_CONFIG_H
#define METER_CONFIG_H

/* 改输出只看这两项；频率单位Hz，波形可选SINE、TRIANGLE、SQUARE。 */
#define METER_DDS_HZ              20000U /* DDS期望频率；保留用户当前输出设定，单位Hz。 */
#define METER_DDS_WAVE            AD9833_WAVE_SINE

/* 一帧采4096对样本；250ms启动一帧，不是每250ms采一个点。 */
#define METER_SAMPLE_COUNT        4096U /* 一帧4096对S1/S2；200kS/s时采样历时20.48ms。 */
#define METER_UPDATE_PERIOD_MS    250U /* 两次开始测量至少隔250ms，忙时不重入；不是采样间隔。 */
#define METER_CAPTURE_TIMEOUT_MS  100U /* 只给主采样等待用的超时；不是RMS或Vpp滤波窗口。 */

/* 启动默认值；有效帧会使用内部参考得到的实时VDDA，不用逐次手改这里。 */
#define METER_VDDA_V              3.3f

/* capture.c按VREFINT典型1.200V估计本帧VDDA；不使用额外通道倍率。 */

#endif
