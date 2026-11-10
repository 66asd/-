#pragma once
#include "stm32f1xx_hal.h"
#include "config.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "ad9833.h"
#include "ad9959.h"
static inline void check(HAL_StatusTypeDef status)
{
    if (status != HAL_OK) Error_Handler();
}
void screen_uart_irq(void);
void sample_start(void);
void sample_changed(void);
const uint32_t *sample_take(void);
typedef struct {
    int hz, phase, percent, wave, increment, span, step, dwell, sweep, offset;
    uint32_t next;
} Source;
extern Source source[2];
void settings_init(void);
void settings_key(unsigned page,unsigned id);
void settings_tick(void);
typedef struct {unsigned hz,rms_mv,pp_mv,first_q8,period_q8,clipped; } Reading;
typedef enum { FIT_OK, FIT_CLIP, FIT_UNLOCKED, FIT_DISTORTED } FitStatus;
typedef struct {
    Reading reading[2];
    uint16_t raw_min, raw_max;
    float residual_codes;
    FitStatus status;
    uint32_t sequence;
    uint8_t trace[2][PLOT_WIDTH];
} DisplayFrame;
extern unsigned vdda_mv;
/* One diagnostic result per completed software frame; sequence written last. */
typedef struct {
    uint32_t min_code, max_code, mean_code, reference_code;
    float raw_pp_mv, fitted_pp_mv;
    uint32_t sequence;
} AdcDiagnostic;
extern volatile AdcDiagnostic adc_diagnostic;
extern unsigned reference_code;
void measurement_tick(void);
void measurement_changed(void);
const DisplayFrame *measurement_acquire(void);
void screen_init(void);
void screen_tick(void);
void screen_invalidate(void);
