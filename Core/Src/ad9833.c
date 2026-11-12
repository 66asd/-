#include "ad9833.h"
#include "config.h"
#include "spi.h"

static unsigned active_bank;

static HAL_StatusTypeDef write_word(uint16_t word)
{
    uint8_t bytes[] = {(uint8_t)(word >> 8), (uint8_t)word};
    HAL_StatusTypeDef result;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    result = HAL_SPI_Transmit(&hspi2, bytes, sizeof(bytes), 5U);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    return result;
}

HAL_StatusTypeDef AD9833_Init(void)
{
    HAL_Delay(10U);
    active_bank = 0;
    return write_word(0x2100U);
}

HAL_StatusTypeDef AD9833_SetOutput(uint32_t hz, unsigned degrees, AD9833_Wave wave)
{
    static const uint16_t wave_bits[] = {0, 2, 0x28};
    if (hz >= MCLK9833_HZ / 2 || degrees >= 360 || (unsigned)wave > AD9833_SQUARE)
        return HAL_ERROR;

    /* Prepare the inactive frequency and phase pair; select both with one control word. */
    unsigned bank = active_bank ^ 1U;
    uint16_t frequency_reg = bank ? 0x8000U : 0x4000U;
    uint16_t phase_reg = bank ? 0xe000U : 0xc000U;
    uint32_t ftw = (uint32_t)((((uint64_t)hz << 28) + MCLK9833_HZ / 2) / MCLK9833_HZ);
    uint16_t phase = (uint16_t)((degrees * 4096U + 180U) / 360U);
    uint16_t select = 0x2000U | (bank ? 0x0c00U : 0U) | wave_bits[wave];
    HAL_StatusTypeDef result = write_word(frequency_reg | (ftw & 0x3fffU));
    if (result == HAL_OK) result = write_word(frequency_reg | (ftw >> 14));
    if (result == HAL_OK) result = write_word(phase_reg | phase);
    if (result == HAL_OK) result = write_word(select);
    if (result == HAL_OK) active_bank = bank;
    return result;
}
