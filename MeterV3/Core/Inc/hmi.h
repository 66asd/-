#ifndef HMI_H
#define HMI_H
#include "main.h"
#include "signal_math.h"

void Hmi_Init(void);
uint8_t Hmi_TakeRequest(void); /* 取走一次屏幕测量请求，没有请求时返回0。 */
/* t4/t5为两路AC RMS，t6频率，t7相位，t9/t11为两路Vpp；电压显示mV。 */
void Hmi_Show(const MeterResult *r);
#endif
