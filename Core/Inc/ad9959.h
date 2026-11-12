#pragma once
#include "main.h"

HAL_StatusTypeDef AD9959_Init(void);
HAL_StatusTypeDef AD9959_SetCH0(uint32_t hz, unsigned degrees, unsigned percent);
