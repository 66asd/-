#ifndef APP_H
#define APP_H
#include "main.h"
#include "signal_math.h"
void App_Init(void);
void App_Poll(void);
extern MeterResult g_last_result;
extern volatile uint32_t g_comm_errors;
#endif
