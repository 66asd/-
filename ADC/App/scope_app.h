#ifndef SCOPE_APP_H
#define SCOPE_APP_H
#include <stdint.h>
typedef struct {
    uint32_t processed_blocks, dma_overruns, hmi_errors, displayed_frames;
    uint32_t max_process_cycles;
    uint32_t sample_rate_hz, analysis_rate_hz[2], rate_changes, analyzed_frames;
    uint32_t uart_tx_bytes, uart_rx_bytes;
    uint8_t adc_fault, hmi_page;
} ScopeDiagnostics;
extern volatile ScopeDiagnostics g_scope_diagnostics;
int ScopeApp_Init(void);
void ScopeApp_Process(void);
#endif
