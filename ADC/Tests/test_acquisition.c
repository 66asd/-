#include "main.h"
#include "scope_acquisition.h"
volatile uint32_t test_failed_line,test_case,test_passed;
volatile float test_details[8];
#ifdef __CC_ARM
#define STOP() __breakpoint(0)
#else
#define STOP() __asm volatile("bkpt #0")
#endif
#define CHECK(x) do { if(!(x)) { test_failed_line=__LINE__; STOP(); for(;;){} } } while(0)
ADC_TypeDef fake_adc1,fake_adc2;
RCC_TypeDef fake_rcc={1u};
ADC_HandleTypeDef hadc1={ADC1},hadc2={ADC2};
DMA_HandleTypeDef hdma_adc1;
static TIM_TypeDef timer;
TIM_HandleTypeDef htim3={&timer,{0u,359u}};
static uint32_t mode_value,calibrations,mode_restores;
static int timer_on,irq_enabled;
static uint32_t *dma;
static uint32_t copy[SCOPE_DMA_HALF_WORDS];
uint32_t HAL_RCC_GetPCLK1Freq(void) { return 36000000u; }
HAL_StatusTypeDef HAL_TIM_Base_Stop(TIM_HandleTypeDef *h) { (void)h; timer_on=0; return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *h)
{
    (void)h; CHECK(hdma_adc1.busy && mode_value==ADC_DUALMODE_REGSIMULT && irq_enabled);
    CHECK(timer.CNT==0u && (timer.SR&TIM_FLAG_UPDATE)==0u); timer_on=1; return HAL_OK;
}
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *h) { h->Instance->CR2=0u; return HAL_OK; }
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *h)
{
    CHECK(!timer_on); ++calibrations; h->Instance->CR2=1u; return HAL_OK;
}
HAL_StatusTypeDef HAL_ADCEx_MultiModeStop_DMA(ADC_HandleTypeDef *h)
{
    (void)h; CHECK(!timer_on);
    /* These two details reproduce the real F1 HAL, not an always-success stub. */
    mode_value=0u; fake_adc1.CR2=fake_adc2.CR2=0u;
    if(!hdma_adc1.busy) return HAL_ERROR;
    hdma_adc1.busy=0; return HAL_OK;
}
HAL_StatusTypeDef HAL_ADCEx_MultiModeConfigChannel(ADC_HandleTypeDef *h,ADC_MultiModeTypeDef *m)
{
    (void)h; CHECK(!fake_adc1.CR2 && !fake_adc2.CR2 && !timer_on);
    mode_value=m->Mode; ++mode_restores; return HAL_OK;
}
HAL_StatusTypeDef HAL_ADCEx_MultiModeStart_DMA(ADC_HandleTypeDef *h,uint32_t *p,uint32_t n)
{
    (void)h; CHECK(!hdma_adc1.busy && !timer_on && mode_value==ADC_DUALMODE_REGSIMULT);
    CHECK(n==1024u); dma=p; hdma_adc1.busy=1; fake_adc1.CR2=fake_adc2.CR2=1u; return HAL_OK;
}
void HAL_NVIC_DisableIRQ(int irq) { (void)irq; irq_enabled=0; }
void HAL_NVIC_EnableIRQ(int irq) { (void)irq; irq_enabled=1; }
void HAL_NVIC_ClearPendingIRQ(int irq) { (void)irq; }
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *h);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *h);
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *h);
void test_start(void)
{
    static const uint32_t rates[]={5000u,10000u,20000u,50000u,100000u,200000u,500000u};
    uint32_t i,j,index; int gap;
    ++test_case; CHECK(Acquisition_Start()); CHECK(calibrations==2u);
    CHECK(Acquisition_Rate()==500000u && timer.ARR==143u);
    for(i=0u;i<7u;++i) {
        ++test_case;
        CHECK(Acquisition_SetRate(rates[i]));
        CHECK(timer.ARR==72000000u/rates[i]-1u && Acquisition_Rate()==rates[i]);
        CHECK(Acquisition_CopyLatest(copy,&index,&gap)==0);
        for(j=0u;j<1024u;++j) dma[j]=j|((4095u-j)<<16);
        HAL_ADC_ConvHalfCpltCallback(&hadc1);
        CHECK(Acquisition_CopyLatest(copy,&index,&gap)==1 && !gap && index==0u);
        CHECK(copy[20]==(20u|(4075u<<16)));
        HAL_ADC_ConvCpltCallback(&hadc1); HAL_ADC_ConvHalfCpltCallback(&hadc1);
        CHECK(Acquisition_CopyLatest(copy,&index,&gap)==1 && gap && index==1024u);
        CHECK(mode_value==ADC_DUALMODE_REGSIMULT);
    }
    ++test_case; CHECK(!Acquisition_SetRate(1000000u) && timer_on);
    CHECK(!Acquisition_SetRate(0u) && timer_on);
    CHECK(Acquisition_Overruns()==7u && mode_restores==8u);
    HAL_ADC_ErrorCallback(&hadc2); CHECK(Acquisition_HasFault());
    CHECK(Acquisition_Start()); CHECK(calibrations==4u && !Acquisition_HasFault());
    test_passed=1u; STOP(); for(;;){}
}
#ifdef __CC_ARM
void SystemInit(void) {}
int main(void) { test_start(); return 0; }
#endif
