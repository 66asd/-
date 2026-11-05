#ifndef METER_CONFIG_H
#define METER_CONFIG_H
#define METER_SAMPLE_COUNT 2048U
#define METER_CAPTURE_TIMEOUT_MS 100U
#define METER_DDS_HZ 5000U
#define METER_UPDATE_PERIOD_MS 250U /* automatic start-to-start interval; about 4 updates/s */
#if METER_UPDATE_PERIOD_MS == 0
#error "METER_UPDATE_PERIOD_MS must be nonzero"
#endif
#define METER_VDDA_V 3.300f  /* measure VDDA; never use scope probe multiplier here */
#define METER_GAIN_S1 1.000f /* physical input transfer ratio, calibrate using known DC */
#define METER_GAIN_S2 1.000f
#define METER_OFFSET_S1_V 0.000f
#define METER_OFFSET_S2_V 0.000f
#endif
