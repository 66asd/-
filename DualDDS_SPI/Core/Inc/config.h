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
/* ADC-pin mV to original mV, independently measured divider gains. */
#define GAIN1_PERMILLE 1000
#define GAIN2_PERMILLE 1000
#define FS_HZ 100000UL
#define HALF_PAIRS 1024U
#define PLOT_WIDTH 479U
#define DEFAULT_WINDOW 400U
#define MAX_WINDOW 800U
