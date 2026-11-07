#ifndef CAPTURE_H
#define CAPTURE_H
#include "main.h"
#include "meter_config.h"

typedef enum
{
    CAPTURE_IDLE, /* 还没开始采样。 */
    CAPTURE_PENDING, /* 定时器和DMA正在完成这一帧。 */
    CAPTURE_DONE, /* 主样本和前后参考均成功，可计算本帧结果。 */
    CAPTURE_FAILED /* 本帧失败，硬件已停止，可由下一帧重新尝试。 */
} CaptureState;

HAL_StatusTypeDef Capture_Init(void);
HAL_StatusTypeDef Capture_Begin(void); /* 启动一帧即返回，不在这里等待采满。 */
CaptureState Capture_Poll(void);      /* 主循环反复查询是否完成。 */
/* 只有DONE后可用；下一次Begin会覆盖这些数据，必须先算完当前帧。 */
const uint32_t *Capture_Data(void);
float Capture_SampleRate(void);
/* 当前帧实测VDDA；只有DONE后使用，参考测量失败时不输出旧量程。 */
float Capture_ReferenceVoltage(void);
#endif
