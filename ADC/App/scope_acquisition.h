#ifndef SCOPE_ACQUISITION_H
#define SCOPE_ACQUISITION_H
#include <stdint.h>
#include "scope_config.h"
int Acquisition_Start(void);
int Acquisition_SetRate(uint32_t rate);
int Acquisition_SetLongSampling(int enabled);
uint16_t Acquisition_SampleCyclesX2(void);
uint32_t Acquisition_Rate(void);
int Acquisition_CopyLatest(uint32_t *copy, uint32_t *first_index, int *gap);
int Acquisition_HasFault(void);
uint32_t Acquisition_Overruns(void);
#endif
