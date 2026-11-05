#include "capture.h"
#include "adc.h"
#include "tim.h"
static uint32_t samples[METER_SAMPLE_COUNT];
static volatile uint8_t dma_done, dma_error;
static CaptureState state = CAPTURE_IDLE;
static uint32_t started_ms;
/* Deliberately do not call F1 MultiModeStop_DMA: it aborts even completed DMA
   and clears DUALMOD. Stop ADCs explicitly and abort only an active transfer. */
static HAL_StatusTypeDef stop_capture(void)
{
    HAL_StatusTypeDef s = HAL_OK;
    if (HAL_TIM_Base_Stop(&htim3) != HAL_OK)
        s = HAL_ERROR;
    if (HAL_ADC_Stop(&hadc1) != HAL_OK)
        s = HAL_ERROR;
    if (HAL_ADC_Stop(&hadc2) != HAL_OK)
        s = HAL_ERROR;
    CLEAR_BIT(hadc1.Instance->CR2, ADC_CR2_DMA);
    if (HAL_DMA_GetState(hadc1.DMA_Handle) == HAL_DMA_STATE_BUSY &&
        HAL_DMA_Abort(hadc1.DMA_Handle) != HAL_OK)
        s = HAL_ERROR;
    __HAL_DMA_DISABLE(hadc1.DMA_Handle);
    return s;
}
HAL_StatusTypeDef Capture_Init(void)
{
    if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
        return HAL_ERROR;
    if (HAL_ADCEx_Calibration_Start(&hadc2) != HAL_OK)
        return HAL_ERROR;
    return stop_capture();
}
HAL_StatusTypeDef Capture_Begin(void)
{
    ADC_MultiModeTypeDef mode = {0};
    if (state == CAPTURE_PENDING)
        return HAL_BUSY;
    if (stop_capture() != HAL_OK)
        return HAL_ERROR;
    dma_done = 0;
    dma_error = 0;
    mode.Mode = ADC_DUALMODE_REGSIMULT;
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &mode) != HAL_OK)
        return HAL_ERROR;
    if (HAL_ADC_Start(&hadc2) != HAL_OK)
        goto fail;
    if (HAL_ADCEx_MultiModeStart_DMA(&hadc1, samples, METER_SAMPLE_COUNT) != HAL_OK)
        goto fail;
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
    state = CAPTURE_PENDING;
    started_ms = HAL_GetTick();
    if (HAL_TIM_Base_Start(&htim3) != HAL_OK)
        goto fail;
    return HAL_OK;
fail:
    (void)stop_capture();
    state = CAPTURE_FAILED;
    return HAL_ERROR;
}
CaptureState Capture_Poll(void)
{
    if (state != CAPTURE_PENDING)
        return state;
    if (dma_error || HAL_GetTick() - started_ms >= METER_CAPTURE_TIMEOUT_MS)
    {
        (void)stop_capture();
        state = CAPTURE_FAILED;
    }
    else if (dma_done)
    {
        uint32_t remaining = __HAL_DMA_GET_COUNTER(hadc1.DMA_Handle);
        state = (stop_capture() == HAL_OK && remaining == 0U) ? CAPTURE_DONE : CAPTURE_FAILED;
    }
    return state;
}
const uint32_t *Capture_Data(void)
{
    return samples;
}
float Capture_SampleRate(void)
{
    uint32_t clock = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0U)
        clock *= 2U;
    return (float)clock / (float)(htim3.Init.Prescaler + 1U) / (float)(htim3.Init.Period + 1U);
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *h)
{
    if (h->Instance == ADC1)
    {
        __HAL_TIM_DISABLE(&htim3);
        dma_done = 1;
    }
}
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *h)
{
    if (h->Instance == ADC1)
    {
        __HAL_TIM_DISABLE(&htim3);
        dma_error = 1;
    }
}
