#include "scope_acquisition.h"
#include "adc.h"
#include "tim.h"
#include "main.h"
static volatile uint32_t dma_words[SCOPE_DMA_WORDS];
static volatile uint32_t produced, fault;
static uint32_t consumed, overruns;

int Acquisition_Start(void)
{
    uint32_t clock = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0u) clock *= 2u;
    if (clock / (htim3.Init.Prescaler + 1u) / (htim3.Init.Period + 1u) != SCOPE_SAMPLE_RATE_HZ) return 0;
    if (HAL_TIM_Base_Stop(&htim3) != HAL_OK) return 0;
    if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) return 0;
    if (HAL_ADCEx_Calibration_Start(&hadc2) != HAL_OK) return 0;
    produced = consumed = overruns = fault = 0u;
    __HAL_TIM_SET_COUNTER(&htim3, 0u);
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
    if (HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *)dma_words, SCOPE_DMA_WORDS) != HAL_OK) return 0;
    if (HAL_TIM_Base_Start(&htim3) != HAL_OK) {
        (void)HAL_ADCEx_MultiModeStop_DMA(&hadc1);
        return 0;
    }
    return 1;
}

int Acquisition_CopyLatest(uint32_t *copy, uint32_t *first_index, int *gap)
{
    uint32_t seq = produced, i, offset;
    if (seq == consumed) return 0;
    *gap = seq - consumed != 1u;
    if (*gap) overruns += seq - consumed - 1u;
    offset = ((seq - 1u) & 1u) * SCOPE_DMA_HALF_WORDS;
    __DMB();
    for (i = 0u; i < SCOPE_DMA_HALF_WORDS; ++i) copy[i] = dma_words[offset + i];
    __DMB();
    consumed = seq;
    if (produced != seq) {
        /* The DMA crossed a boundary during copy: never process a torn block. */
        ++overruns;
        return -1;
    }
    *first_index = (seq - 1u) * SCOPE_DMA_HALF_WORDS;
    return 1;
}
int Acquisition_HasFault(void) { return fault != 0u; }
uint32_t Acquisition_Overruns(void) { return overruns; }
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *adc)
{
    if (adc->Instance == ADC1) { __DMB(); ++produced; }
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *adc)
{
    if (adc->Instance == ADC1) { __DMB(); ++produced; }
}
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *adc)
{
    if (adc->Instance == ADC1 || adc->Instance == ADC2) fault = 1u;
}
