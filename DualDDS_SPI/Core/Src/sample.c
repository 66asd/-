#include "system_app.h"
static volatile uint32_t dma[2*HALF_PAIRS];
static uint32_t snapshot[HALF_PAIRS];
static volatile uint32_t generation;
static volatile unsigned completed;
static uint32_t consumed;
static unsigned discard;
volatile uint32_t missed_frames, raced_frames;
static void finished(unsigned half) { completed=half; __DMB(); ++generation; }
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *a){if(a==&hadc1) finished(0);}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *a){if(a==&hadc1) finished(1);}
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *a){(void)a;check(HAL_ERROR);}
void sample_start(void) {
    check(HAL_ADCEx_Calibration_Start(&hadc1)); check(HAL_ADCEx_Calibration_Start(&hadc2));
    check(HAL_ADC_Start(&hadc2));
    check(HAL_ADCEx_MultiModeStart_DMA(&hadc1,(uint32_t*)dma,2*HALF_PAIRS));
    check(HAL_TIM_Base_Start(&htim3));
}
void sample_changed(void){consumed=generation; discard=2;}
static unsigned safe(unsigned h){
    uint32_t left=DMA1_Channel1->CNDTR;
    return left && (h==0 ? left<=HALF_PAIRS : left>HALF_PAIRS);
}
const uint32_t *sample_take(void){
    uint32_t g=generation, delta=g-consumed;
    unsigned h=completed,i;
    if(!delta) return 0;
    missed_frames+=delta-1; consumed=g;
    if(discard>=delta){discard-=delta;return 0;}
    discard=0;
    if(!safe(h) || generation!=g) return 0;
    for(i=0;i<HALF_PAIRS;i++) snapshot[i]=dma[h*HALF_PAIRS+i];
    __DMB();
    if(!safe(h) || generation!=g){++raced_frames;return 0;}
    return snapshot;
}
