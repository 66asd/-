#ifndef VOFA_RAW_H
#define VOFA_RAW_H
#include <stdint.h>
typedef int (*VofaSend)(void *context, const uint8_t *data, uint16_t size);
typedef struct {
    uint32_t sample_rate_hz, segment_id, samples_sent, segments_sent, errors;
    uint8_t enabled;
} VofaDiagnostics;
extern volatile VofaDiagnostics g_vofa_diagnostics;
void VofaRaw_Init(VofaSend send, void *context);
void VofaRaw_Enable(int enabled);
int VofaRaw_Enabled(void);
/* 1 = VOFA, 2 = HMI!, 0 = incomplete/unrecognized command. */
int VofaRaw_Command(uint8_t byte);
void VofaRaw_Offer(const uint32_t *packed, uint16_t count, uint32_t sample_rate);
void VofaRaw_Process(uint32_t now);
void VofaRaw_TxComplete(void);
int VofaRaw_TimedOut(uint32_t now);
void VofaRaw_TransportError(void);
#endif
