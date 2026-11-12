#pragma once
#include "main.h"

typedef enum {
    AD9833_SINE, AD9833_TRIANGLE, AD9833_SQUARE
} AD9833_Wave;

HAL_StatusTypeDef AD9833_Init(void);
HAL_StatusTypeDef AD9833_SetOutput(uint32_t hz, unsigned degrees, AD9833_Wave wave);
