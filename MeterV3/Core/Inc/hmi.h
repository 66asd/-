#ifndef HMI_H
#define HMI_H
#include "main.h"
#include "signal_math.h"

typedef enum
{
    HMI_REQUEST_NONE = 0,
    HMI_REQUEST_MEASURE = 1,
    HMI_REQUEST_DIAGNOSTIC = 2
} HmiRequest;

HAL_StatusTypeDef Hmi_Init(void);
/* 多次按钮请求只保留最新一条，避免测量任务排成长队。 */
HmiRequest Hmi_TakeRequest(void);
/* 正常t4/t5=两路AC RMS，t6=频率，t7=相位；t9/t11始终为两路Vpp。 */
HAL_StatusTypeDef Hmi_Show(const MeterResult *r, uint8_t diagnostic);
extern volatile uint32_t g_hmi_rx_errors;
#endif
