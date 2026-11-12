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
extern unsigned sample_hz;
int sample_rate(unsigned hz);
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
extern Reading reading[2];
extern uint8_t trace[2][PLOT_WIDTH];
extern unsigned vdda_mv;
void reset_span(void);
int span_ready(const uint32_t *pairs);
void measure(const uint32_t *pairs);
void screen_init(void);
void screen_tick(void);
void screen_invalidate(void);
void scope_key(unsigned id);
