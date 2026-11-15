#include "scope_acquisition.h"
#include "adc.h"
#include "tim.h"
#include "main.h"
static volatile uint32_t dma_words[SCOPE_DMA_WORDS];
static volatile uint32_t produced, fault;
static uint32_t consumed, overruns, actual_rate;
static uint8_t running;
static uint8_t long_sampling;
static uint16_t sample_cycles_x2=15u;

static uint32_t sampling_time(uint32_t rate, int longer, uint16_t *cycles_x2)
{
    if(!longer || rate>200000u) { *cycles_x2=15u; return ADC_SAMPLETIME_7CYCLES_5; }
    if(rate>100000u) { *cycles_x2=83u; return ADC_SAMPLETIME_41CYCLES_5; }
    if(rate>20000u) { *cycles_x2=143u; return ADC_SAMPLETIME_71CYCLES_5; }
    *cycles_x2=479u; return ADC_SAMPLETIME_239CYCLES_5;
}
uint16_t Acquisition_SampleCyclesX2(void) { return sample_cycles_x2; }
int Acquisition_SetLongSampling(int enabled)
{
    uint16_t next;
    long_sampling=(uint8_t)(enabled!=0);
    (void)sampling_time(actual_rate,long_sampling,&next);
    if(!running || next==sample_cycles_x2) return 1;
    return Acquisition_SetRate(actual_rate);
}

uint32_t Acquisition_Rate(void) { return actual_rate; }
int Acquisition_SetRate(uint32_t rate)
{
    uint32_t clock = HAL_RCC_GetPCLK1Freq(), ticks;
    ADC_MultiModeTypeDef mode = {0};
    ADC_ChannelConfTypeDef channel = {0};
    uint16_t next_cycles;
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0u) clock *= 2u;
    if (rate < 5000u || rate > SCOPE_SAMPLE_RATE_HZ) return 0;
    ticks = (clock + rate / 2u) / rate;
    if (ticks < 144u || ticks > 65536u) return 0;
    if (HAL_TIM_Base_Stop(&htim3) != HAL_OK) return 0;
    if (running) {
        if (HAL_ADCEx_MultiModeStop_DMA(&hadc1) != HAL_OK) return 0;
        running = 0u;
    } else {
        if (HAL_ADC_Stop(&hadc1) != HAL_OK || HAL_ADC_Stop(&hadc2) != HAL_OK) return 0;
    }
    HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
    /* ADCs are disabled: the update event must not create a conversion. */
    htim3.Init.Prescaler = 0u; htim3.Init.Period = ticks - 1u;
    htim3.Instance->PSC = 0u;
    __HAL_TIM_SET_AUTORELOAD(&htim3, ticks - 1u);
    htim3.Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_SET_COUNTER(&htim3, 0u);
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
    __HAL_DMA_CLEAR_FLAG(&hdma_adc1, DMA_FLAG_GL1);
    HAL_NVIC_ClearPendingIRQ(DMA1_Channel1_IRQn);
    produced = consumed = 0u;
    /* F1 MultiModeStop_DMA clears DUALMOD; restore it on every restart. */
    mode.Mode = ADC_DUALMODE_REGSIMULT;
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc1,&mode) != HAL_OK) {
        HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn); return 0;
    }
    /* Both sample windows stay equal, preserving dual-ADC relative phase. */
    channel.Rank=ADC_REGULAR_RANK_1;
    channel.SamplingTime=sampling_time(rate,long_sampling,&next_cycles);
    channel.Channel=ADC_CHANNEL_0;
    if(HAL_ADC_ConfigChannel(&hadc1,&channel)!=HAL_OK) {
        HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn); return 0;
    }
    channel.Channel=ADC_CHANNEL_1;
    if(HAL_ADC_ConfigChannel(&hadc2,&channel)!=HAL_OK) {
        HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn); return 0;
    }
    /* This F1 HAL Init leaves EXTTRIG clear; MultiModeStart only arms the master. */
    if (HAL_ADC_Start(&hadc2) != HAL_OK) {
        HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn); return 0;
    }
    if (HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *)dma_words, SCOPE_DMA_WORDS) != HAL_OK) {
        HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn); return 0;
    }
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    if (HAL_TIM_Base_Start(&htim3) != HAL_OK) {
        (void)HAL_ADCEx_MultiModeStop_DMA(&hadc1); return 0;
    }
    actual_rate = clock / ticks;
    sample_cycles_x2=next_cycles;
    running = 1u;
    return 1;
}

int Acquisition_Start(void)
{
    long_sampling=0u;
    if (HAL_TIM_Base_Stop(&htim3) != HAL_OK) return 0;
    if (running) {
        if (HAL_ADCEx_MultiModeStop_DMA(&hadc1) != HAL_OK) return 0;
        running = 0u;
    }
    if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) return 0;
    if (HAL_ADCEx_Calibration_Start(&hadc2) != HAL_OK) return 0;
    produced = consumed = overruns = fault = 0u;
    return Acquisition_SetRate(SCOPE_SAMPLE_RATE_HZ);
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
