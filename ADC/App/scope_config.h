#ifndef SCOPE_CONFIG_H
#define SCOPE_CONFIG_H

#define SCOPE_SAMPLE_RATE_HZ       500000u /* Maximum and acquisition-search rate. */
#define SCOPE_MIN_FREQUENCY_HZ     50u
#ifndef SCOPE_TARGET_POINTS
#define SCOPE_TARGET_POINTS        16u /* 1 kHz uses 20 kS/s instead of 100 kS/s. */
#endif
#define SCOPE_PROBE_INTERVAL_MS    1000u
#define SCOPE_RATE_HOLD_MS         300u
#define SCOPE_REFERENCE_MV         3300u /* Measure VDDA and calibrate this value. */
#define SCOPE_ADC_FULL_SCALE       4095u
#define SCOPE_DMA_WORDS            1024u
#define SCOPE_DMA_HALF_WORDS       (SCOPE_DMA_WORDS / 2u)
#define SCOPE_TRACE_SAMPLES        1024u
#define SCOPE_DISPLAY_CYCLES       4u
#define SCOPE_DISPLAY_INTERVAL_MS  150u
#define SCOPE_HMI_PAGE             3u
#define SCOPE_HMI_WAVE_A_ID        1u
#define SCOPE_HMI_WAVE_B_ID        15u
#define SCOPE_HMI_AUTO_ID          16u
#define SCOPE_HMI_STOP_ID          17u
#define SCOPE_WAVE_A_WIDTH         242u
#define SCOPE_WAVE_B_WIDTH         239u
#define SCOPE_WAVE_A_HEIGHT        213u
#define SCOPE_WAVE_B_HEIGHT        211u

#endif
