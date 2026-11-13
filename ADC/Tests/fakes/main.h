#ifndef TEST_MAIN_H
#define TEST_MAIN_H
#include <stdint.h>
typedef struct { uint32_t DEMCR; } TestCoreDebug;
typedef struct { uint32_t CYCCNT, CTRL; } TestDwt;
extern TestCoreDebug test_core_debug;
extern TestDwt test_dwt;
#define CoreDebug (&test_core_debug)
#define DWT (&test_dwt)
#define CoreDebug_DEMCR_TRCENA_Msk 1u
#define DWT_CTRL_CYCCNTENA_Msk 1u
uint32_t HAL_GetTick(void);
void Error_Handler(void);
#endif
