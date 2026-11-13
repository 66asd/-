#ifndef SCOPE_APP_H
#define SCOPE_APP_H
#include <stdint.h>
typedef struct {
    uint32_t processed_blocks, dma_overruns, hmi_errors, displayed_frames;
    uint32_t max_process_cycles;
    uint8_t adc_fault;
} ScopeDiagnostics;
extern volatile ScopeDiagnostics g_scope_diagnostics;
int ScopeApp_Init(void);
void ScopeApp_Process(void);
#endif
