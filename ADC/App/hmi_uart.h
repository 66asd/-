#ifndef HMI_UART_H
#define HMI_UART_H
#include "hmi_protocol.h"
int HmiUart_Init(void);
void HmiUart_Process(void);
HmiProtocol *HmiUart_Protocol(void);
#endif
