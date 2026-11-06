/**
  ******************************************************************************
  * @file    stm32f1xx_hal_conf.h
  * @brief   HAL configuration file.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */


#ifndef __STM32F1xx_HAL_CONF_H
#define __STM32F1xx_HAL_CONF_H

/* 只启用采样、DDS控制、串口及系统初始化用到的模块。 */
#define HAL_MODULE_ENABLED
#define HAL_ADC_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_SPI_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED

/* HAL时钟驱动需要这些常量；定义数值不表示开启对应振荡器。 */
#define HSE_VALUE             8000000U
#define HSE_STARTUP_TIMEOUT   100U
#define HSI_VALUE             8000000U
#define LSI_VALUE             40000U
#define LSE_VALUE             32768U
#define LSE_STARTUP_TIMEOUT   5000U
#define VDD_VALUE             3300U
#define TICK_INT_PRIORITY     15U
#define USE_RTOS              0U
#define PREFETCH_ENABLE       1U
#define USE_SPI_CRC           0U

/* 使用固定回调函数，不启用运行中注册回调的另一套接口。 */
#define USE_HAL_ADC_REGISTER_CALLBACKS   0U
#define USE_HAL_SPI_REGISTER_CALLBACKS   0U
#define USE_HAL_TIM_REGISTER_CALLBACKS   0U
#define USE_HAL_UART_REGISTER_CALLBACKS  0U

#include "stm32f1xx_hal_rcc.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_dma.h"
#include "stm32f1xx_hal_cortex.h"
#include "stm32f1xx_hal_adc.h"
#include "stm32f1xx_hal_flash.h"
#include "stm32f1xx_hal_pwr.h"
#include "stm32f1xx_hal_spi.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32f1xx_hal_uart.h"

#define assert_param(expr) ((void)0U)
#endif
