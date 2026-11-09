#include "system_app.h"
ADC_HandleTypeDef hadc1, hadc2;
DMA_HandleTypeDef hdma_adc1;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart1;
void check(HAL_StatusTypeDef status) {
    if(status != HAL_OK) { __disable_irq(); for(;;) {} }
}
static void port(GPIO_TypeDef *p,uint16_t pins,uint32_t mode) {
    GPIO_InitTypeDef g={0};
    g.Pin=pins; g.Mode=mode; g.Pull=GPIO_NOPULL; g.Speed=GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(p,&g);
}
static void adc(ADC_HandleTypeDef *a,ADC_TypeDef *instance,uint32_t channel,uint32_t trigger) {
    ADC_ChannelConfTypeDef c={0};
    a->Instance=instance;
    a->Init.ScanConvMode=ADC_SCAN_DISABLE;
    a->Init.ContinuousConvMode=DISABLE;
    a->Init.DiscontinuousConvMode=DISABLE;
    a->Init.ExternalTrigConv=trigger;
    a->Init.DataAlign=ADC_DATAALIGN_RIGHT;
    a->Init.NbrOfConversion=1;
    check(HAL_ADC_Init(a));
    c.Channel=channel; c.Rank=ADC_REGULAR_RANK_1; c.SamplingTime=ADC_SAMPLETIME_55CYCLES_5;
    check(HAL_ADC_ConfigChannel(a,&c));
}
void board_init(void) {
    RCC_OscInitTypeDef osc={0};
    RCC_ClkInitTypeDef clk={0};
    RCC_PeriphCLKInitTypeDef per={0};
    TIM_MasterConfigTypeDef master={0};
    ADC_MultiModeTypeDef dual={0};
    osc.OscillatorType=RCC_OSCILLATORTYPE_HSE; osc.HSEState=RCC_HSE_ON;
    osc.PLL.PLLState=RCC_PLL_ON; osc.PLL.PLLSource=RCC_PLLSOURCE_HSE;
    osc.HSEPredivValue=RCC_HSE_PREDIV_DIV1; osc.PLL.PLLMUL=RCC_PLL_MUL9;
    check(HAL_RCC_OscConfig(&osc));
    clk.ClockType=RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource=RCC_SYSCLKSOURCE_PLLCLK; clk.AHBCLKDivider=RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider=RCC_HCLK_DIV2; clk.APB2CLKDivider=RCC_HCLK_DIV1;
    check(HAL_RCC_ClockConfig(&clk,FLASH_LATENCY_2));
    per.PeriphClockSelection=RCC_PERIPHCLK_ADC; per.AdcClockSelection=RCC_ADCPCLK2_DIV6;
    check(HAL_RCCEx_PeriphCLKConfig(&per));
    __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE(); __HAL_AFIO_REMAP_SWJ_NOJTAG();
    __HAL_RCC_ADC1_CLK_ENABLE(); __HAL_RCC_ADC2_CLK_ENABLE(); __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE(); __HAL_RCC_USART1_CLK_ENABLE();
    port(GPIOA,GPIO_PIN_0|GPIO_PIN_1,GPIO_MODE_ANALOG);
    GPIOB->BSRR=GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_6;
    port(GPIOB,GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_5|
         GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15,GPIO_MODE_OUTPUT_PP);
    port(GPIOA,GPIO_PIN_9,GPIO_MODE_AF_PP); port(GPIOA,GPIO_PIN_10,GPIO_MODE_INPUT);
    huart1.Instance=USART1; huart1.Init.BaudRate=115200;
    huart1.Init.WordLength=UART_WORDLENGTH_8B; huart1.Init.StopBits=UART_STOPBITS_1;
    huart1.Init.Parity=UART_PARITY_NONE; huart1.Init.Mode=UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl=UART_HWCONTROL_NONE; huart1.Init.OverSampling=UART_OVERSAMPLING_16;
    check(HAL_UART_Init(&huart1));
    hdma_adc1.Instance=DMA1_Channel1; hdma_adc1.Init.Direction=DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc=DMA_PINC_DISABLE; hdma_adc1.Init.MemInc=DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment=DMA_PDATAALIGN_WORD;
    hdma_adc1.Init.MemDataAlignment=DMA_MDATAALIGN_WORD;
    hdma_adc1.Init.Mode=DMA_CIRCULAR; hdma_adc1.Init.Priority=DMA_PRIORITY_HIGH;
    check(HAL_DMA_Init(&hdma_adc1)); __HAL_LINKDMA(&hadc1,DMA_Handle,hdma_adc1);
    adc(&hadc1,ADC1,ADC_CHANNEL_0,ADC_EXTERNALTRIGCONV_T3_TRGO);
    adc(&hadc2,ADC2,ADC_CHANNEL_1,ADC_SOFTWARE_START);
    dual.Mode=ADC_DUALMODE_REGSIMULT; check(HAL_ADCEx_MultiModeConfigChannel(&hadc1,&dual));
    htim3.Instance=TIM3; htim3.Init.Period=719; htim3.Init.Prescaler=0;
    htim3.Init.CounterMode=TIM_COUNTERMODE_UP; htim3.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    check(HAL_TIM_Base_Init(&htim3)); master.MasterOutputTrigger=TIM_TRGO_UPDATE;
    master.MasterSlaveMode=TIM_MASTERSLAVEMODE_DISABLE;
    check(HAL_TIMEx_MasterConfigSynchronization(&htim3,&master));
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn,0,0); HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    HAL_NVIC_SetPriority(USART1_IRQn,1,0); HAL_NVIC_EnableIRQ(USART1_IRQn);
}
void SysTick_Handler(void){HAL_IncTick();}
void DMA1_Channel1_IRQHandler(void){HAL_DMA_IRQHandler(&hdma_adc1);}
