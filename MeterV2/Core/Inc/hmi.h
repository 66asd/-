#ifndef HMI_H
#define HMI_H
#include "main.h"
#include "signal_math.h"
HAL_StatusTypeDef Hmi_Init(void);
typedef enum
{
    HMI_REQUEST_NONE = 0,
    HMI_REQUEST_MEASURE = 1,
    HMI_REQUEST_DIAGNOSTIC = 2
} HmiRequest;
/* Foreground consumer; coalesces queued events to the latest complete frame. */
HmiRequest Hmi_TakeRequest(void);
HAL_StatusTypeDef Hmi_Show(const MeterResult *r, uint8_t diagnostic);
extern volatile uint32_t g_hmi_rx_errors;
#endif
