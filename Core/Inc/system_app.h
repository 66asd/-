#pragma once
#include "stm32f1xx_hal.h"
#include "config.h"
extern ADC_HandleTypeDef hadc1, hadc2;
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1;
void board_init(void);
void check(HAL_StatusTypeDef status);
void sample_start(void);
void sample_changed(void);
const uint32_t *sample_take(void);
typedef struct {
    int hz, phase, percent, wave, increment, span, step, dwell, sweep, offset;
    uint32_t next;
} Source;
extern Source source[2];
void ad9833_init(void);
void ad9833_write(const Source *s);
void ad9959_init(void);
void ad9959_write(const Source *s);
void settings_init(void);
void settings_key(unsigned page,unsigned id);
void settings_tick(void);
typedef struct {unsigned hz,rms_mv,pp_mv,first_q8,period_q8; } Reading;
extern Reading reading[2];
extern uint8_t trace[2][PLOT_WIDTH];
void measure(const uint32_t *pairs);
void screen_init(void);
void screen_tick(void);
void screen_invalidate(void);
