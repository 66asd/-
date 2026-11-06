#include "ad9833.h"
#include "spi.h"

#define AD9833_CTRL_B28 0x2000U
#define AD9833_CTRL_RESET 0x0100U
#define AD9833_CTRL_OPBITEN 0x0020U
#define AD9833_CTRL_DIV2 0x0008U
#define AD9833_CTRL_TRIANGLE 0x0002U
#define AD9833_FREQ0 0x4000U
#define AD9833_PHASE0 0xC000U

/*
 * SPI 配置必须为：
 * 8位、MSB first、CPOL High、CPHA 1 Edge。
 *
 * 一个16位命令拆成两个字节，高字节先发。
 * 两个字节发送期间 FSYNC 始终保持低电平。
 */
static HAL_StatusTypeDef AD9833_Write16(uint16_t word)
{
    uint8_t tx[2];
    HAL_StatusTypeDef status;

    tx[0] = (uint8_t)(word >> 8);
    tx[1] = (uint8_t)(word & 0xFFU);

    HAL_GPIO_WritePin(AD9833_FSYNC_GPIO_Port, AD9833_FSYNC_Pin, GPIO_PIN_RESET);

    status = HAL_SPI_Transmit(&hspi1, tx, 2U, 100U);

    HAL_GPIO_WritePin(AD9833_FSYNC_GPIO_Port, AD9833_FSYNC_Pin, GPIO_PIN_SET);

    return status;
}

HAL_StatusTypeDef AD9833_SetOutput(uint32_t frequency_hz, AD9833_WaveType wave)
{
    uint32_t tuning_word;
    uint16_t control;
    HAL_StatusTypeDef status;

    /*
     * 限制为参考时钟的一半以下。
     * 实际需要良好波形时，应进一步远离此上限。
     */
    if (frequency_hz >= (AD9833_MCLK_HZ / 2UL))
    {
        return HAL_ERROR;
    }

    control = AD9833_CTRL_B28;

    switch (wave)
    {
    case AD9833_WAVE_SINE:
        /* OPBITEN=0, MODE=0 */
        break;

    case AD9833_WAVE_TRIANGLE:
        /* OPBITEN=0, MODE=1 */
        control |= AD9833_CTRL_TRIANGLE;
        break;

    case AD9833_WAVE_SQUARE:
        /* OPBITEN=1, DIV2=1, MODE=0 */
        control |= AD9833_CTRL_OPBITEN | AD9833_CTRL_DIV2;
        break;

    default:
        return HAL_ERROR;
    }

    /*
     * 使用64位整数计算并四舍五入，防止32位乘法溢出。
     * STM32F103没有硬件浮点单元，这里无需使用浮点。
     */
    tuning_word =
        (uint32_t)((((uint64_t)frequency_hz << 28) + (AD9833_MCLK_HZ / 2UL)) / AD9833_MCLK_HZ);

    /* 1. 保持复位，同时选择28位频率写入模式 */
    status = AD9833_Write16(AD9833_CTRL_B28 | AD9833_CTRL_RESET);
    if (status != HAL_OK)
    {
        return status;
    }

    /* 2. 先写低14位 */
    status = AD9833_Write16(AD9833_FREQ0 | (uint16_t)(tuning_word & 0x3FFFUL));
    if (status != HAL_OK)
    {
        return status;
    }

    /* 3. 再写高14位 */
    status = AD9833_Write16(AD9833_FREQ0 | (uint16_t)((tuning_word >> 14) & 0x3FFFUL));
    if (status != HAL_OK)
    {
        return status;
    }

    /* 4. PHASE0 = 0 */
    status = AD9833_Write16(AD9833_PHASE0);
    if (status != HAL_OK)
    {
        return status;
    }

    /* 5. 解除复位，开始输出选定波形 */
    return AD9833_Write16(control);
}

void AD9833_Init(void)
{
    HAL_GPIO_WritePin(AD9833_FSYNC_GPIO_Port, AD9833_FSYNC_Pin, GPIO_PIN_SET);

    /* 给模块上电和参考晶振启动预留时间 */
    HAL_Delay(10U);

    /* 这里只准备硬件。频率和波形由App_Init统一设置，避免前后两次设置互相覆盖。 */
}
