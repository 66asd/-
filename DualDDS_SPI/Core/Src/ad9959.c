#include "ad9959.h"
#include "spi.h"
#include "config.h"

static HAL_StatusTypeDef write_register(uint8_t address, uint32_t value, unsigned width)
{
    uint8_t bytes[5] = {address};
    for (unsigned i = 0; i < width; ++i)
        bytes[width - i] = (uint8_t)(value >> (8U * i));

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_StatusTypeDef result = HAL_SPI_Transmit(&hspi1, bytes, width + 1U, 5U);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    return result;
}

static void latch(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    for (volatile unsigned i = 0; i < 32; ++i) __NOP();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
}

HAL_StatusTypeDef AD9959_Init(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_Delay(1);
    uint32_t pll = PLL9959 == 1 ? 0 : PLL9959 << 18;
    if (SYS9959_HZ > 255000000UL) pll |= 0x800000UL;
    HAL_StatusTypeDef result = write_register(0, 0x20, 1); /* CSR selects CH1 */
    if (result == HAL_OK) result = write_register(1, pll, 3);
    if (result != HAL_OK) return result;
    latch();
    HAL_Delay(5);
    return write_register(3, 0x301, 3); /* full-scale DAC, sine */
}

HAL_StatusTypeDef AD9959_SetCH0(uint32_t hz, unsigned degrees, unsigned percent)
{
    if (hz >= SYS9959_HZ / 2 || degrees >= 360 || percent > 100) return HAL_ERROR;
    uint32_t ftw = (uint32_t)((((uint64_t)hz << 32) + SYS9959_HZ / 2) / SYS9959_HZ);
    uint16_t phase = (uint16_t)((degrees * 16384U + 180U) / 360U);
    uint16_t amplitude = 0x1000U | ((percent * 1023U + 50U) / 100U);

    /* All three shadow registers take effect together. */
    HAL_StatusTypeDef result = write_register(4, ftw, 4);
    if (result == HAL_OK) result = write_register(5, phase, 2);
    if (result == HAL_OK) result = write_register(6, amplitude, 3);
    if (result == HAL_OK) latch();
    return result;
}
