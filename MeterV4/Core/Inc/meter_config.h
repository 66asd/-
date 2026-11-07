#ifndef METER_CONFIG_H
#define METER_CONFIG_H

/* 改输出只看这两项；频率单位Hz，波形可选SINE、TRIANGLE、SQUARE。 */
#define METER_DDS_HZ              500U
#define METER_DDS_WAVE            AD9833_WAVE_SINE

/* 一帧采2048对样本；250ms启动一帧，不是每250ms采一个点。 */
#define METER_SAMPLE_COUNT        2048U
#define METER_UPDATE_PERIOD_MS    250U
#define METER_CAPTURE_TIMEOUT_MS  100U

/* 启动默认值；有效帧会使用内部参考得到的实时VDDA，不用逐次手改这里。 */
#define METER_VDDA_V              3.2f

/* 由实测VDDA=3.2V和64次内部参考码1530.234375算得；不是典型1.2V，也不取自目标Vpp。 */
#define METER_VREFINT_CAL_V       1.195495605469f

#endif
