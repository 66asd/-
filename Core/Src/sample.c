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
unsigned vdda_mv;
unsigned sample_hz = FS_HZ;
static uint32_t reference_cal_sum, reference_next;
static unsigned reference_cal_mv;
volatile uint32_t reference_errors, reference_store_errors, reference_updates;
volatile uint32_t reference_sum;
#define CAL_TAG 0x56445231UL
typedef struct { uint32_t tag, sum, mv, checksum; } ReferenceCalibration;
static int reference_valid(uint32_t sum) {
    return sum >= REFERENCE_SAMPLES*1000U && sum <= REFERENCE_SAMPLES*2300U;
}
unsigned reference_voltage(uint32_t calibration_sum, uint32_t sum, unsigned calibration_mv) {
    if (!reference_valid(sum) || !reference_valid(calibration_sum)) return 0;
    unsigned mv=(unsigned)(((uint64_t)calibration_mv*calibration_sum+sum/2U)/sum);
    return mv>=2400U && mv<=3600U ? mv : 0;
}
static void load_calibration(uint32_t sum) {
    const ReferenceCalibration *saved=(const ReferenceCalibration *)ADC_CAL_ADDRESS;
    if(saved->tag==CAL_TAG && reference_valid(saved->sum) &&
       saved->mv>=2400U && saved->mv<=3600U &&
       saved->checksum==(CAL_TAG ^ saved->sum ^ saved->mv)) {
        reference_cal_sum=saved->sum; reference_cal_mv=saved->mv;
        return;
    }
    reference_cal_sum=sum;
    reference_cal_mv=ADC_VDDA_CAL_MV;
    /* A nonblank unrecognized record is never erased automatically. */
    for(unsigned i=0;i<sizeof(*saved)/4U;++i) {
        if(((const uint32_t *)saved)[i]!=0xFFFFFFFFUL) {
            ++reference_store_errors; return;
        }
    }
    if(HAL_FLASH_Unlock()!=HAL_OK){++reference_store_errors;return;}
    HAL_StatusTypeDef status=HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,ADC_CAL_ADDRESS+4U,sum);
    if(status==HAL_OK)status=HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,ADC_CAL_ADDRESS+8U,reference_cal_mv);
    if(status==HAL_OK)status=HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,ADC_CAL_ADDRESS+12U,CAL_TAG^sum^reference_cal_mv);
    /* Commit marker last: incomplete writes cannot become a valid calibration. */
    if(status==HAL_OK)status=HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,ADC_CAL_ADDRESS,CAL_TAG);
    if(HAL_FLASH_Lock()!=HAL_OK || status!=HAL_OK)++reference_store_errors;
}
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
    for (i = 0; i < REFERENCE_SAMPLES+2U && status == HAL_OK; ++i)
    {
        status = HAL_ADC_Start(&hadc1);
        if (status != HAL_OK) break;
        status = HAL_ADC_PollForConversion(&hadc1, 5);
        if (status != HAL_OK) break;
        uint32_t value=HAL_ADC_GetValue(&hadc1) & 0xFFFU;
        if(i>=2U)*sum+=value; /* Discard channel-switch settling conversions. */
    }
    if (HAL_ADC_Stop(&hadc1) != HAL_OK)
        status = HAL_ERROR;
    /* 成功或失败都恢复原通道和TIM触发，不能让下一帧误采内部参考。 */
    hadc1.Instance->SMPR1 = saved_sampling;
    hadc1.Instance->SQR3 = saved_sequence;
    hadc1.Instance->CR1 = saved_cr1;
    hadc1.Instance->CR2 = saved_cr2;
    if (!reference_valid(*sum)) status = HAL_ERROR;
    return status;
}


static void start_external(void) {
    /* HAL's multimode stop clears DUALMOD; restore it on every restart. */
    ADC_MultiModeTypeDef mode = {0};
    mode.Mode = ADC_DUALMODE_REGSIMULT;
    check(HAL_ADCEx_MultiModeConfigChannel(&hadc1, &mode));
    __HAL_TIM_SET_COUNTER(&htim3,0U);
    check(HAL_ADC_Start(&hadc2));
    check(HAL_ADCEx_MultiModeStart_DMA(&hadc1,(uint32_t*)dma,2*HALF_PAIRS));
    sample_changed();
    check(HAL_TIM_Base_Start(&htim3));
}
static void stop_external(void) {
    check(HAL_TIM_Base_Stop(&htim3));
    check(HAL_ADCEx_MultiModeStop_DMA(&hadc1));
    check(HAL_ADC_Stop(&hadc2));
}
static void schedule_reference(uint32_t now) {
    /* Slow timebases must publish a settled DMA record before the next pause. */
    uint32_t interval=(HALF_PAIRS*4000U+sample_hz-1U)/sample_hz;
    if(interval<ADC_REFERENCE_MS)interval=ADC_REFERENCE_MS;
    reference_next=now+interval;
}
void sample_start(void) {
    check(HAL_ADCEx_Calibration_Start(&hadc1)); check(HAL_ADCEx_Calibration_Start(&hadc2));
    uint32_t sum = 0;
    SET_BIT(ADC1->CR2, ADC_CR2_TSVREFE);
    HAL_Delay(1);
    check(measure_reference(&sum));
    load_calibration(sum);
    reference_sum=sum;
    vdda_mv=reference_voltage(reference_cal_sum,sum,reference_cal_mv);
    check(vdda_mv?HAL_OK:HAL_ERROR);
    schedule_reference(HAL_GetTick());
    start_external();
}
void sample_tick(void) {
    uint32_t now=HAL_GetTick(), sum=0;
    if((int32_t)(now-reference_next)<0)return;
    schedule_reference(now);
    /* Record boundary: never inject a slow VREF conversion into an external frame.
       The current partial DMA record is discarded; each published frame is uniform. */
    stop_external();
    if(measure_reference(&sum)==HAL_OK) {
        unsigned mv=reference_voltage(reference_cal_sum,sum,reference_cal_mv);
        if(mv){vdda_mv=mv;reference_sum=sum;++reference_updates;}
        else ++reference_errors;
    } else ++reference_errors; /* Keep the last valid VDDA on timeout/bad codes. */
    start_external();
}
void sample_changed(void){reset_span();consumed=generation; discard=2;}
int sample_rate(unsigned hz){
    uint32_t clock = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0U) clock *= 2U;
    if (hz < FS_MIN_HZ) hz = FS_MIN_HZ;
    if (hz > FS_MAX_HZ) hz = FS_MAX_HZ;
    uint32_t ticks = (clock + hz / 2U) / hz;
    unsigned actual = (clock + ticks / 2U) / ticks;
    if (actual > sample_hz * 9U / 10U && actual < sample_hz * 11U / 10U) return 0;
    stop_external();
    __HAL_TIM_SET_AUTORELOAD(&htim3, ticks - 1U);
    __HAL_TIM_SET_COUNTER(&htim3, 0U);
    sample_hz = actual;
    schedule_reference(HAL_GetTick());
    start_external();
    return 1;
}
static unsigned safe(unsigned h){
    uint32_t left=DMA1_Channel1->CNDTR;
    return left && (h==0 ? left<=HALF_PAIRS : left>HALF_PAIRS);
}
const uint32_t *sample_take(void){
    uint32_t g=generation, delta=g-consumed;
    unsigned h=completed,i;
    if(!delta) return 0;
    if(delta>1) reset_span();
    missed_frames+=delta-1; consumed=g;
    if(discard>=delta){discard-=delta;return 0;}
    discard=0;
    if(!safe(h) || generation!=g){reset_span();return 0;}
    for(i=0;i<HALF_PAIRS;i++) snapshot[i]=dma[h*HALF_PAIRS+i];
    __DMB();
    if(!safe(h) || generation!=g){++raced_frames;reset_span();return 0;}
    return snapshot;
}
