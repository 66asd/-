#pragma once
#define MCLK9833_HZ 25000000UL
#define REF9959_HZ 25000000UL
#define PLL9959 20U /* 1 = bypass, or 4..20 */
#define SYS9959_HZ (REF9959_HZ*PLL9959)
#if (PLL9959 != 1 && (PLL9959 < 4 || PLL9959 > 20)) || SYS9959_HZ > 500000000UL
#error Invalid AD9959 clock
#endif
#define DEFAULT_F1 1000
#define DEFAULT_F2 2000
#define F_MIN 300
#define F_MAX 10000
#define F_STEP 100
#define PHASE_STEP 15
#define AMP_STEP 5
#define SWEEP_SPAN 1000
#define SWEEP_STEP 100
#define SWEEP_MS 200
#define SPAN_DELTA 100
#define STEP_DELTA 10
#define SPEED_DELTA_MS 10
#define DWELL_MIN 50
#define DWELL_MAX 2000
#define VDDA_MV 3300
/* First calibration only: measured supply, persisted with its averaged VREF code. */
#define ADC_VDDA_CAL_MV 3020U
#define ADC_VREFINT_CAL_MV 1200U
#define ADC_CAL_ADDRESS 0x0800FC00UL /* Reserved last 1KB of the 64KB C8 flash. */
#define ADC_REFERENCE_MS 1000U
#define DDS_ENABLED 0
/* ADC-pin mV to original mV, independently measured divider gains. */
#define GAIN1_PERMILLE 1000
#define GAIN2_PERMILLE 1000
#define FS_HZ 600000UL
#define FS_MAX_HZ 600000UL
#define FS_MIN_HZ 2000UL
#define SCOPE_MAX_HZ 100000UL
#define SCOPE_PROBE_MS 2000U
#define HALF_PAIRS 1024U
#define PLOT_WIDTH 241U
#define PLOT_WIDTH2 238U
#define DEFAULT_WINDOW 400U
/* 示波器式自动时基：横轴约 4 个周期（要求 3~5 个）。 */
#define SCOPE_CYCLES 4U
/* Verified in DualDDS2.HMI: b1=16, b2=17, s0=1, s1=15. */
#define SCOPE_KEY_AUTO 16U
#define SCOPE_KEY_STOP 17U
