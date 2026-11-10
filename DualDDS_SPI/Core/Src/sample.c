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
#define REFERENCE_SAMPLES 64U
unsigned vdda_mv, reference_code;
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
    for (i = 0; i < REFERENCE_SAMPLES + 16U && status == HAL_OK; ++i)
    {
        status = HAL_ADC_Start(&hadc1);
        if (status != HAL_OK) break;
        status = HAL_ADC_PollForConversion(&hadc1, 5);
        if (status != HAL_OK) break;
        uint32_t value = HAL_ADC_GetValue(&hadc1) & 0xFFFU;
        if (i >= 16U) *sum += value;
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


void sample_start(void) {
    check(HAL_ADCEx_Calibration_Start(&hadc1)); check(HAL_ADCEx_Calibration_Start(&hadc2));
    uint32_t sum = 0;
    SET_BIT(ADC1->CR2, ADC_CR2_TSVREFE);
    HAL_Delay(1);
    check(measure_reference(&sum));
    reference_code = (sum + REFERENCE_SAMPLES/2U)/REFERENCE_SAMPLES;
    vdda_mv = (VREFINT_TYP_MV * 4096U * REFERENCE_SAMPLES + sum / 2) / sum;
    check(HAL_ADC_Start(&hadc2));
    check(HAL_ADCEx_MultiModeStart_DMA(&hadc1,(uint32_t*)dma,2*HALF_PAIRS));
    check(HAL_TIM_Base_Start(&htim3));
}
void sample_changed(void){measurement_changed();consumed=generation; discard=2;}
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
    if(!safe(h) || generation!=g){return 0;}
    for(i=0;i<HALF_PAIRS;i++) snapshot[i]=dma[h*HALF_PAIRS+i];
    __DMB();
    if(!safe(h) || generation!=g){++raced_frames;return 0;}
    return snapshot;
}
