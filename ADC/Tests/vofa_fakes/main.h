#ifndef VOFA_TEST_MAIN_H
#define VOFA_TEST_MAIN_H
#include <stdint.h>
#define __DMB() ((void)0)
#define __get_PRIMASK() 0u
#define __disable_irq() ((void)0)
#define __set_PRIMASK(x) ((void)(x))
uint32_t HAL_GetTick(void);
#endif
