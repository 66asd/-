#ifndef AD9833_H
#define AD9833_H

#include "main.h"

/* 必须与模块实际参考时钟一致 */
#define AD9833_MCLK_HZ 25000000UL

typedef enum
{
    AD9833_WAVE_SINE,
    AD9833_WAVE_TRIANGLE,
    AD9833_WAVE_SQUARE
} AD9833_WaveType;

/* 准备FSYNC并等待模块上电，输出参数随后由App_Init统一设置。 */
void AD9833_Init(void);

/* 本接口使用整数 Hz；每次配置会短暂复位输出 */
HAL_StatusTypeDef AD9833_SetOutput(uint32_t frequency_hz, AD9833_WaveType wave);

#endif
