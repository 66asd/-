#ifndef HMI_PROTOCOL_H
#define HMI_PROTOCOL_H
#include <stdint.h>
#include "scope_config.h"
#define HMI_EVENT_AUTO 1u
#define HMI_EVENT_STOP 2u
#define HMI_EVENT_PAGE 4u
typedef struct {
    uint32_t frequency_tenths, rms_mv, vpp_mv;
    uint8_t frequency_valid;
} HmiReading;
typedef struct {
    uint8_t a[SCOPE_WAVE_A_WIDTH], b[SCOPE_WAVE_B_WIDTH];
    HmiReading reading[2];
} HmiSnapshot;
typedef int (*HmiSend)(void *context, const uint8_t *data, uint16_t size);
typedef struct {
    HmiSnapshot snapshot;
    HmiSend send;
    void *context;
    uint8_t command[320], rx[16];
    volatile uint8_t transmitting;
    uint8_t rx_size, page, paused, events, state, channel, status_dirty, failed;
    uint32_t ready_count, done_count, ready_before, done_before;
    uint32_t deadline, last_query, errors;
} HmiProtocol;
void HmiProtocol_Init(HmiProtocol *h, HmiSend send, void *context, uint32_t now);
void HmiProtocol_Receive(HmiProtocol *h, uint8_t byte);
void HmiProtocol_Process(HmiProtocol *h, uint32_t now);
void HmiProtocol_TxComplete(HmiProtocol *h);
void HmiProtocol_TransportError(HmiProtocol *h);
int HmiProtocol_CanSubmit(const HmiProtocol *h);
int HmiProtocol_Submit(HmiProtocol *h, const HmiSnapshot *snapshot);
uint8_t HmiProtocol_TakeEvents(HmiProtocol *h);
#endif
