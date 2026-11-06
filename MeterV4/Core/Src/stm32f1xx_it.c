#include "stm32f1xx_it.h"
#include "adc.h"
#include "usart.h"

/* 其余异常沿用启动文件的默认处理，这里只实现实际用到的3个中断。 */
void SysTick_Handler(void)
{
    HAL_IncTick(); /* HAL延时、采集超时和自动刷新都使用这个毫秒计数。 */
}

void DMA1_Channel1_IRQHandler(void)
{
    HAL_DMA_IRQHandler(hadc1.DMA_Handle);
}

void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}
