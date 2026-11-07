#include "capture.h"
#include "adc.h"
#include "tim.h"

/* DMA把同一时刻的S1/S2放在一个32位字里，共占8192字节。 */
static uint32_t samples[METER_SAMPLE_COUNT];
static volatile uint8_t dma_done, dma_error;
static CaptureState state = CAPTURE_IDLE;
static uint32_t started_ms;
static uint32_t reference_before_sum, reference_after_sum;
static float reference_voltage;

#define REFERENCE_SAMPLES 64U

static void stop_capture(void)
{
    /* 先断触发源，避免清理ADC时又开始一次转换。 */
    HAL_TIM_Base_Stop(&htim3);
    HAL_ADC_Stop(&hadc1);
    HAL_ADC_Stop(&hadc2);
    CLEAR_BIT(hadc1.Instance->CR2, ADC_CR2_DMA);

    /* Normal DMA正常采满后已经结束，只有仍在搬运时才需要中止。 */
    if (HAL_DMA_GetState(hadc1.DMA_Handle) == HAL_DMA_STATE_BUSY)
    {
        HAL_DMA_Abort(hadc1.DMA_Handle);
    }
    __HAL_DMA_DISABLE(hadc1.DMA_Handle);
}

static HAL_StatusTypeDef fail_capture(void)
{
    stop_capture();
    state = CAPTURE_FAILED;
    return HAL_ERROR;
}

/* 只在TIM和DMA停止时读内部参考，不打断S1/S2同步采样。 */
static HAL_StatusTypeDef measure_reference(uint32_t *sum)
{
    uint32_t i;
    HAL_StatusTypeDef status = HAL_OK;
    *sum = 0;
    for (i = 0; i < REFERENCE_SAMPLES; ++i)
    {
        status = HAL_ADCEx_InjectedStart(&hadc1);
        if (status != HAL_OK) break;
        status = HAL_ADCEx_InjectedPollForConversion(&hadc1, 5);
        if (status != HAL_OK) break;
        *sum += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    }
    if (HAL_ADCEx_InjectedStop(&hadc1) != HAL_OK)
        status = HAL_ERROR;
    if (*sum == 0U) status = HAL_ERROR;
    return status;
}

HAL_StatusTypeDef Capture_Init(void)
{
    ADC_InjectionConfTypeDef ref = {0};
    /* 上电各校准一次，减少ADC自身偏移；这不代替外部电压校准。 */
    if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
        return HAL_ERROR;
    if (HAL_ADCEx_Calibration_Start(&hadc2) != HAL_OK)
        return HAL_ERROR;
    stop_capture();
    ref.InjectedChannel = ADC_CHANNEL_VREFINT;
    ref.InjectedRank = ADC_INJECTED_RANK_1;
    ref.InjectedNbrOfConversion = 1;
    ref.InjectedSamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    ref.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &ref) != HAL_OK)
        return HAL_ERROR;
    SET_BIT(hadc1.Instance->CR2, ADC_CR2_TSVREFE);
    HAL_Delay(1); /* 首次使能后等参考源稳定，后续保持开启。 */
    return HAL_OK;
}

HAL_StatusTypeDef Capture_Begin(void)
{
    ADC_MultiModeTypeDef mode = {0};
    if (state == CAPTURE_PENDING)
        return HAL_BUSY;

    stop_capture();
    /* 清掉上一帧的通知，防止刚开始新帧就被误认成已完成。 */
    dma_done = 0;
    dma_error = 0;

    /* F1双ADC要先准备从机，再开主机DMA，最后才允许定时器发触发。 */
    mode.Mode = ADC_DUALMODE_REGSIMULT;
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &mode) != HAL_OK)
        return fail_capture();
    if (HAL_ADC_Start(&hadc2) != HAL_OK)
        return fail_capture();
    if (measure_reference(&reference_before_sum) != HAL_OK)
        return fail_capture();
    if (HAL_ADCEx_MultiModeStart_DMA(&hadc1, samples, METER_SAMPLE_COUNT) != HAL_OK)
        return fail_capture();

    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
    started_ms = HAL_GetTick();
    state = CAPTURE_PENDING;
    if (HAL_TIM_Base_Start(&htim3) != HAL_OK)
        return fail_capture();
    return HAL_OK;
}

CaptureState Capture_Poll(void)
{
    uint32_t remaining;
    if (state != CAPTURE_PENDING)
        return state;

    /* 采不到完整数据就退出本帧，不能让主循环永远等下去。 */
    if (dma_error || HAL_GetTick() - started_ms >= METER_CAPTURE_TIMEOUT_MS)
    {
        fail_capture();
        return state;
    }
    if (!dma_done)
        return CAPTURE_PENDING;

    remaining = __HAL_DMA_GET_COUNTER(hadc1.DMA_Handle);
    stop_capture();
    if (remaining != 0U || measure_reference(&reference_after_sum) != HAL_OK)
    {
        fail_capture();
        return state;
    }
    /* 用帧前、帧后的参考平均值，减少供电在采样期间变化带来的偏差。 */
    reference_voltage = METER_VREFINT_CAL_V * (4096.0f * REFERENCE_SAMPLES * 2U) /
                        (reference_before_sum + reference_after_sum);
    if (reference_voltage < 2.0f || reference_voltage > 3.6f)
    {
        fail_capture();
        return state;
    }
    state = CAPTURE_DONE;
    return state;
}

const uint32_t *Capture_Data(void)
{
    /* 返回原数组，避免复制8KB。下一帧开始前，这些数据保持不变。 */
    return samples;
}

float Capture_ReferenceVoltage(void)
{
    return reference_voltage;
}

float Capture_SampleRate(void)
{
    uint32_t clock = HAL_RCC_GetPCLK1Freq();
    /* APB1分频不为1时，TIM3时钟是APB1时钟的两倍。 */
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0U)
        clock *= 2U;
    return (float)clock / (htim3.Init.Prescaler + 1U) / (htim3.Init.Period + 1U);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *h)
{
    if (h->Instance == ADC1)
    {
        /* 中断只停触发并通知前台，计算和显示留给App_Poll。 */
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
