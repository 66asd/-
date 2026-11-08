/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

#define DDS_MCLK_HZ 25000000u
#define TIMER_HZ    72000000u
#define DWELL_MS    100u
#define FRAME_COUNT 2048u

static uint16_t samples[FRAME_COUNT];
volatile uint32_t sweep_set_hz;
volatile uint32_t measured_millihz;  /* Zero until this plateau has a valid frame. */
static uint32_t applied_at, sample_rate, power_at;
static uint16_t words[5];
static unsigned word_count, sent;
static uint8_t active_bank;
static enum { CAL_POWER, CAL_RESET, CAL_RUN, CAL_DONE } cal;
static enum { ARM_FRAME, CAPTURING, FRAME_DONE } acquisition;

/* Fit t[k] = intercept + period*k to all rising crossings, using Q8 sample times. */
uint32_t estimate_millihz(const uint16_t *data, uint32_t fs)
{
    uint32_t i, low = 4095, high = 0, armed = 0, count = 0;
    uint32_t mid, hysteresis, last = 0, initial_period = 0;
    uint64_t sum_t = 0, sum_kt = 0, sum_k, sum_kk, denominator;
    for (i = 0; i < FRAME_COUNT; ++i) {
        if (data[i] < low) low = data[i];
        if (data[i] > high) high = data[i];
    }
    if (high - low < 32) return 0;
    mid = (high + low) / 2;
    hysteresis = (high - low) / 10;
    for (i = 1; i < FRAME_COUNT; ++i) {
        uint32_t a = data[i-1], b = data[i];
        if (a + hysteresis < mid) armed = 1;
        if (armed && a <= mid && b > mid) {
            uint32_t t = (i-1)*256 + ((mid-a)*256 + (b-a)/2)/(b-a);
            uint32_t interval = t - last;
            if (count == 1) initial_period = interval;
            if (count >= 2 && (interval < initial_period*3/4 ||
                               interval > initial_period*5/4)) return 0;
            sum_t += t;
            sum_kt += (uint64_t)count*t;
            last = t;
            ++count;
            armed = 0;
        }
    }
    if (count < 4 || count > 180) return 0;
    sum_k = count*(count-1)/2;
    sum_kk = count*(count-1)*(2*count-1)/6;
    denominator = count*sum_kt - sum_k*sum_t;
    /* Two stages keep the millihertz numerator within uint64_t. */
    {
        uint64_t scaled = (count*sum_kk - sum_k*sum_k)*fs*256;
        return (uint32_t)((scaled/denominator)*1000 +
               ((scaled%denominator)*1000 + denominator/2)/denominator);
    }
}

static uint32_t requested;
static void queue_frequency(uint32_t hz, unsigned first)
{
    uint32_t tuning = (uint32_t)((((uint64_t)hz << 28) + DDS_MCLK_HZ/2)/DDS_MCLK_HZ);
    uint16_t address;
    requested = hz;
    active_bank = first ? 0 : (active_bank ^ 1u);
    address = active_bank ? 0x8000u : 0x4000u;
    sent = word_count = 0;
    if (first) words[word_count++] = 0x2100u;
    words[word_count++] = address | (tuning & 0x3fffu);
    words[word_count++] = address | (tuning >> 14);
    if (first) words[word_count++] = 0xc000u;
    words[word_count++] = active_bank ? 0x2800u : 0x2000u;
    GPIOA->BRR = AD9833_FSYNC_Pin;
}

static void stop_frame(void)
{
    TIM3->CR1 &= ~TIM_CR1_CEN;
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
}

static void sweep_poll(uint32_t now)
{
    static unsigned descending;
    if (word_count) {
        if (sent < word_count) {
            if (SPI1->SR & SPI_SR_TXE) SPI1->DR = words[sent++];
            return;
        }
        if (!(SPI1->SR & SPI_SR_TXE) || (SPI1->SR & SPI_SR_BSY)) return;
        /* TX-only wiring with the Cube master in full duplex: clear RX overrun. */
        (void)SPI1->DR;
        (void)SPI1->SR;
        GPIOA->BSRR = AD9833_FSYNC_Pin;
        word_count = 0;
        IWDG->KR = 0xaaaau; /* One completed SPI transaction per plateau. */
        applied_at = now;
        sweep_set_hz = requested;
        measured_millihz = 0;
        acquisition = ARM_FRAME;
    } else if (now - applied_at >= DWELL_MS) {
        stop_frame();
        if (sweep_set_hz == 30000) descending = 1;
        if (sweep_set_hz == 100) descending = 0;
        queue_frequency(descending ? sweep_set_hz - 100 : sweep_set_hz + 100, 0);
    }
}

static void meter_poll(uint32_t now)
{
    if (cal != CAL_DONE) {
        switch (cal) {
        case CAL_POWER:
            if (now - power_at >= 2) {
                ADC1->CR2 |= ADC_CR2_RSTCAL;
                cal = CAL_RESET;
            }
            break;
        case CAL_RESET:
            if (!(ADC1->CR2 & ADC_CR2_RSTCAL)) {
                ADC1->CR2 |= ADC_CR2_CAL;
                cal = CAL_RUN;
            }
            break;
        case CAL_RUN:
            if (!(ADC1->CR2 & ADC_CR2_CAL)) cal = CAL_DONE;
            break;
        default: break;
        }
        return;
    }
    if (word_count || now - applied_at < 3) return;
    if (acquisition == ARM_FRAME) {
        sample_rate = sweep_set_hz <= 1900 ? 32000u : 500000u;
        ADC1->CR2 &= ~ADC_CR2_EXTTRIG;
        TIM3->ARR = TIMER_HZ/sample_rate - 1;
        TIM3->EGR = TIM_EGR_UG;
        (void)ADC1->DR;
        DMA1->IFCR = DMA_IFCR_CGIF1;
        DMA1_Channel1->CMAR = (uint32_t)samples;
        DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;
        DMA1_Channel1->CNDTR = FRAME_COUNT;
        DMA1_Channel1->CCR |= DMA_CCR_EN;
        ADC1->CR2 |= ADC_CR2_DMA | ADC_CR2_EXTTRIG;
        TIM3->CR1 |= TIM_CR1_CEN;
        acquisition = CAPTURING;
    } else if (acquisition == CAPTURING) {
        uint32_t flags = DMA1->ISR;
        if (!(flags & (DMA_ISR_TCIF1 | DMA_ISR_TEIF1))) return;
        stop_frame();
        acquisition = FRAME_DONE;
        if (!(flags & DMA_ISR_TEIF1)) measured_millihz = estimate_millihz(samples, sample_rate);
    }
}

/* HSE and PLL start in separate passes so oscillator startup never holds the loop. */
static unsigned boot_poll(void)
{
    static unsigned phase;
    switch (phase) {
    case 0:
        if (!(RCC->CR & RCC_CR_HSERDY)) return 0;
        FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;
        RCC->CFGR = RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL9 |
                    RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_ADCPRE_DIV6;
        RCC->CR |= RCC_CR_PLLON;
        phase = 1;
        return 0;
    case 1:
        if (!(RCC->CR & RCC_CR_PLLRDY)) return 0;
        RCC->CFGR |= RCC_CFGR_SW_PLL;
        phase = 2;
        return 0;
    default:
        if ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) return 0;
        SystemCoreClockUpdate();
        HAL_InitTick(TICK_INT_PRIORITY);
        MX_GPIO_Init();
        MX_DMA_Init();
        MX_ADC1_Init();
        MX_SPI1_Init();
        MX_TIM3_Init();
        ADC1->CR2 |= ADC_CR2_ADON;
        power_at = HAL_GetTick();
        SPI1->CR1 |= SPI_CR1_SPE;
        queue_frequency(100, 1);
        return 1;
    }
}

int main(void)
{
    unsigned ready = 0;
    HAL_Init();
    DBGMCU->CR |= DBGMCU_CR_DBG_IWDG_STOP;
    IWDG->KR = 0xccccu;
    RCC->CR |= RCC_CR_HSEON;
    for (;;) {
        if (!ready) {
            ready = boot_poll();
        } else {
            uint32_t now = HAL_GetTick();
            sweep_poll(now);
            meter_poll(now);
        }
    }
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  NVIC_SystemReset();
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
