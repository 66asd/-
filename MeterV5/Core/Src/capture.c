#include "capture.h"
#include "adc.h"
#include "tim.h"

/* DMA把同一时刻的S1/S2放在一个32位字里，共占16384字节。 */
static uint32_t samples[METER_SAMPLE_COUNT];
/* 中断写、前台读：done表示4096对样本搬完；error表示本轮DMA/ADC发生错误。 */
static volatile uint8_t dma_done, dma_error;
static CaptureState state = CAPTURE_IDLE; /* 当前帧处于未开始、等待、完成或失败中的哪一步。 */
static uint32_t started_ms; /* TIM开始采样时的毫秒计数，超过100ms未完成就退出本帧。 */
/* 帧前、帧后各64次VREFINT原始码的总和；它们不是S1/S2数据。 */
static uint32_t reference_before_sum, reference_after_sum;
static float reference_voltage; /* 本帧由参考码推算出的实际VDDA，单位V；不是典型1.2V。 */

typedef struct
{
    float measured_vdda_v; /* 初次标定时，万用表实际测到的VDDA，单位V。 */
    uint32_t reference_sum; /* 在上述VDDA条件下，内部参考原始码的累加总和。 */
    uint32_t count; /* 上述总和累加了多少次；总和/count才是标定平均码。 */
} ReferenceCalibration;

/* 本板已有的实测记录：VDDA=3.2V，64次参考码总和97935。
 * 它不是目标Vpp的修正量；供电变化时保留此记录，读取新的参考码计算实时VDDA。
 */
static ReferenceCalibration reference_calibration = {3.2f, 97935U, 64U};

#define REFERENCE_SAMPLES 64U /* 每次读参考的次数；一帧前后合计128次。 */

/* 停止触发、ADC和DMA；正常结束或出错退出都用它，避免下一次触发继续改写数组。 */
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

/* 统一结束失败帧：停硬件并记FAILED，主循环就能显示错误而不再等待。 */
static HAL_StatusTypeDef fail_capture(void)
{
    stop_capture();
    state = CAPTURE_FAILED;
    return HAL_ERROR;
}

/* 采样间隙暂用普通软件转换读取参考，避开F103注入触发引起PA0毛刺的勘误。 */
/* 把64次内部参考码累加到*sum；只有成功读满才返回HAL_OK。调用前已停主采样。 */
static HAL_StatusTypeDef measure_reference(uint32_t *sum)
{
    ADC_ChannelConfTypeDef ref = {0}; /* 临时读取内部参考的通道配置，不覆盖外部信号的长期配置。 */
    uint32_t i; /* 本次参考读取的序号，0到63。 */
    uint32_t saved_cr1 = hadc1.Instance->CR1; /* 保存双ADC模式等控制位，结束后恢复。 */
    /* 保存触发/使能配置；ADON和两个START属于运行状态，恢复时不能误启动转换。 */
    uint32_t saved_cr2 = hadc1.Instance->CR2 &
                         ~(ADC_CR2_ADON | ADC_CR2_SWSTART | ADC_CR2_JSWSTART);
    uint32_t saved_sampling = hadc1.Instance->SMPR1; /* 保存高编号通道采样时间寄存器。 */
    uint32_t saved_sequence = hadc1.Instance->SQR3; /* 保存普通序列的通道选择，之后切回PA0。 */
    HAL_StatusTypeDef status; /* 记录本次HAL操作是否成功；任一步失败就停止累加并恢复配置。 */

    /* 调用前两ADC、TIM和DMA均已停止；临时切换不会覆盖S1/S2样本。 */
    CLEAR_BIT(hadc1.Instance->CR1, ADC_CR1_DUALMOD | ADC_CR1_JAUTO);
    MODIFY_REG(hadc1.Instance->CR2,
               ADC_CR2_EXTSEL | ADC_CR2_EXTTRIG | ADC_CR2_DMA | ADC_CR2_CONT |
               ADC_CR2_JEXTTRIG | ADC_CR2_JSWSTART | ADC_CR2_SWSTART,
               ADC_SOFTWARE_START);
    ref.Channel = ADC_CHANNEL_VREFINT;
    ref.Rank = ADC_REGULAR_RANK_1;
    ref.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    status = HAL_ADC_ConfigChannel(&hadc1, &ref);
    *sum = 0; /* 从空总和开始，不能把上一轮参考码混入本轮；sum指向调用者的总和变量。 */
    for (i = 0; i < REFERENCE_SAMPLES && status == HAL_OK; ++i)
    {
        status = HAL_ADC_Start(&hadc1);
        if (status != HAL_OK) break;
        status = HAL_ADC_PollForConversion(&hadc1, 5);
        if (status != HAL_OK) break;
        *sum += HAL_ADC_GetValue(&hadc1) & 0xFFFU;
    }
    if (HAL_ADC_Stop(&hadc1) != HAL_OK)
        status = HAL_ERROR;
    /* 成功或失败都恢复原通道和TIM触发，不能让下一帧误采内部参考。 */
    hadc1.Instance->SMPR1 = saved_sampling;
    hadc1.Instance->SQR3 = saved_sequence;
    hadc1.Instance->CR1 = saved_cr1;
    hadc1.Instance->CR2 = saved_cr2;
    if (*sum == 0U) status = HAL_ERROR;
    return status;
}

/* 上电只调用一次：ADC自校准并打开内部参考源；这里还没有开始采S1/S2。 */
HAL_StatusTypeDef Capture_Init(void)
{
    /* 上电各校准一次，减少ADC自身偏移；这不代替外部电压校准。 */
    if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
        return HAL_ERROR;
    if (HAL_ADCEx_Calibration_Start(&hadc2) != HAL_OK)
        return HAL_ERROR;
    stop_capture();
    SET_BIT(hadc1.Instance->CR2, ADC_CR2_TSVREFE);
    HAL_Delay(1); /* 首次使能后等参考源稳定，后续保持开启。 */
    return HAL_OK;
}

/* 启动一帧：前参考 -> 准备ADC2 -> ADC1+DMA -> TIM3；不在这里等待4096对样本。 */
HAL_StatusTypeDef Capture_Begin(void)
{
    ADC_MultiModeTypeDef mode = {0}; /* 本轮重新确认ADC1为主机、ADC2为同步从机。 */
    if (state == CAPTURE_PENDING)
        return HAL_BUSY;

    stop_capture();
    /* 清掉上一帧的通知，防止刚开始新帧就被误认成已完成。 */
    dma_done = 0;
    dma_error = 0;

    if (measure_reference(&reference_before_sum) != HAL_OK)
        return fail_capture();

    /* F1双ADC要先准备从机，再开主机DMA，最后才允许定时器发触发。 */
    mode.Mode = ADC_DUALMODE_REGSIMULT;
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &mode) != HAL_OK)
        return fail_capture();
    if (HAL_ADC_Start(&hadc2) != HAL_OK)
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

/* 前台检查是否采完；采完后读后参考并更新VDDA，成功才返回DONE供算法使用。 */
CaptureState Capture_Poll(void)
{
    uint32_t remaining; /* DMA还没搬完的32位字数；完成时必须为0。 */
    /* calibration_code是旧标定参考平均码；current_code是本帧128次参考平均码。 */
    float calibration_code, current_code;
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
    calibration_code = (float)reference_calibration.reference_sum / reference_calibration.count;
    current_code = (float)(reference_before_sum + reference_after_sum) / (REFERENCE_SAMPLES * 2U);
    /* VDDA = 标定时实测VDDA × 标定参考码均值 ÷ 本帧参考码均值。 */
    reference_voltage = reference_calibration.measured_vdda_v * calibration_code / current_code;
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
    /* 返回原数组，避免复制16KB。下一帧开始前，这些数据保持不变。 */
    return samples;
}

/* 返回本帧实际VDDA（V）。只能在Capture_Poll返回DONE后用于本帧电压换算。 */
float Capture_ReferenceVoltage(void)
{
    return reference_voltage;
}

/* fs=TIM时钟/(PSC+1)/(ARR+1)，当前为每路200000点/秒，不是DDS输出频率。 */
float Capture_SampleRate(void)
{
    uint32_t clock = HAL_RCC_GetPCLK1Freq(); /* 先取APB1总线频率，再按定时器规则得到TIM3时钟。 */
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
