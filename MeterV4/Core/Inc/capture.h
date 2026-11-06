#ifndef CAPTURE_H
#define CAPTURE_H
#include "main.h"
#include "meter_config.h"

typedef enum
{
    CAPTURE_IDLE,
    CAPTURE_PENDING,
    CAPTURE_DONE,
    CAPTURE_FAILED
} CaptureState;

HAL_StatusTypeDef Capture_Init(void);
HAL_StatusTypeDef Capture_Begin(void); /* 启动一帧即返回，不在这里等待采满。 */
CaptureState Capture_Poll(void);      /* 主循环反复查询是否完成。 */
/* 只有DONE后可用；下一次Begin会覆盖这些数据，必须先算完当前帧。 */
const uint32_t *Capture_Data(void);
float Capture_SampleRate(void);
/* 本帧开始前，用内部参考通道估算的ADC参考电压。 */
float Capture_ReferenceVoltage(void);
#endif
