#ifndef ACQ_TEST_MAIN_H
#define ACQ_TEST_MAIN_H
#include <stdint.h>
typedef int HAL_StatusTypeDef;
#define HAL_OK 0
#define HAL_ERROR 1
typedef struct { uint32_t CR2; } ADC_TypeDef;
typedef struct { ADC_TypeDef *Instance; } ADC_HandleTypeDef;
typedef struct { uint32_t Mode; } ADC_MultiModeTypeDef;
typedef struct { uint32_t PSC,ARR,CNT,SR,EGR; } TIM_TypeDef;
typedef struct { TIM_TypeDef *Instance; struct { uint32_t Prescaler,Period; } Init; } TIM_HandleTypeDef;
typedef struct { int busy; } DMA_HandleTypeDef;
typedef struct { uint32_t CFGR; } RCC_TypeDef;
extern ADC_TypeDef fake_adc1,fake_adc2;
extern RCC_TypeDef fake_rcc;
#define ADC1 (&fake_adc1)
#define ADC2 (&fake_adc2)
#define RCC (&fake_rcc)
#define RCC_CFGR_PPRE1 1u
#define TIM_EGR_UG 1u
#define TIM_FLAG_UPDATE 1u
#define DMA_FLAG_GL1 1u
#define ADC_DUALMODE_REGSIMULT 6u
#define ADC_CR2_ADON 1u
#define ADC_CR2_EXTTRIG (1u<<20)
#define ADC_CR2_SWSTART (1u<<22)
#define DMA1_Channel1_IRQn 0
#define __DMB() do {} while(0)
#define __HAL_TIM_SET_AUTORELOAD(h,n) ((h)->Instance->ARR=(n))
#define __HAL_TIM_SET_COUNTER(h,n) ((h)->Instance->CNT=(n))
#define __HAL_TIM_CLEAR_FLAG(h,n) ((h)->Instance->SR&=~(n))
#define __HAL_DMA_CLEAR_FLAG(h,n) do { (void)(h); (void)(n); } while(0)
uint32_t HAL_RCC_GetPCLK1Freq(void);
HAL_StatusTypeDef HAL_TIM_Base_Stop(TIM_HandleTypeDef *h);
HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *h);
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *h);
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *h);
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *h);
HAL_StatusTypeDef HAL_ADCEx_MultiModeStop_DMA(ADC_HandleTypeDef *h);
HAL_StatusTypeDef HAL_ADCEx_MultiModeStart_DMA(ADC_HandleTypeDef *h,uint32_t *p,uint32_t n);
HAL_StatusTypeDef HAL_ADCEx_MultiModeConfigChannel(ADC_HandleTypeDef *h,ADC_MultiModeTypeDef *mode);
void HAL_NVIC_DisableIRQ(int irq);
void HAL_NVIC_EnableIRQ(int irq);
void HAL_NVIC_ClearPendingIRQ(int irq);
#endif
