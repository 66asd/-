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
HAL_StatusTypeDef Capture_Begin(void);
CaptureState Capture_Poll(void);
/* Valid only after CAPTURE_DONE and until the next Capture_Begin().
 * Packed ADC1 in bits 15:0 and ADC2 in bits 31:16; static 8192-byte storage. */
const uint32_t *Capture_Data(void);
float Capture_SampleRate(void);
#endif
