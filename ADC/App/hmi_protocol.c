#include "hmi_protocol.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
enum { IDLE, CLEAR, ADDT, WAIT_READY, DATA, WAIT_DONE, VALUES, RESYNC };

static int append(HmiProtocol *h, unsigned *used, const char *format, ...)
{
    int n;
    va_list args;
    unsigned available = sizeof(h->command) - *used;
    va_start(args, format);
    n = vsnprintf((char *)h->command + *used, available, format, args);
    va_end(args);
    if (n < 0 || (unsigned)n + 3u >= available) return 0;
    *used += (unsigned)n;
    memset(h->command + *used, 0xff, 3u);
    *used += 3u;
    return 1;
}
static int transmit(HmiProtocol *h, const uint8_t *data, uint16_t size)
{
    /* Set before calling the port: even an immediate completion is safe. */
    h->transmitting = 1u;
    if (h->send(h->context, data, size)) return 1;
    h->transmitting = 0u;
    return 0;
}
static int status_command(HmiProtocol *h, unsigned *used)
{
    /* GBK captions: continue / stop. HMI file itself is never changed. */
    return append(h, used, "b2.txt=\"%s\"", h->paused ? "\xbc\xcc\xd0\xf8" : "\xcd\xa3\xd6\xb9");
}
static int values_command(HmiProtocol *h, unsigned *used)
{
    static const uint8_t fields[2][3] = {{1u,3u,5u},{7u,9u,11u}};
    static const char * const overlays[] = {"t0","t2","t4","t6","t8","t10","b0","b1","b2"};
    unsigned ch;
    for (ch = 0u; ch < 2u; ++ch) {
        const HmiReading *r = &h->snapshot.reading[ch];
        uint32_t f = r->frequency_tenths;
        if (!r->frequency_valid) {
            if (!append(h, used, "t%u.txt=\"--\"", fields[ch][0])) return 0;
        } else if (f >= 10000u) {
            uint32_t khz_tenths = (f + 500u) / 1000u;
            if (!append(h, used, "t%u.txt=\"%lu.%lukHz\"", fields[ch][0],
                        (unsigned long)(khz_tenths / 10u), (unsigned long)(khz_tenths % 10u))) return 0;
        } else if (!append(h, used, "t%u.txt=\"%lu.%luHz\"", fields[ch][0],
                           (unsigned long)(f / 10u), (unsigned long)(f % 10u))) return 0;
        if (!append(h, used, "t%u.txt=\"%lu.%03luV\"", fields[ch][1],
                    (unsigned long)(r->rms_mv / 1000u), (unsigned long)(r->rms_mv % 1000u))) return 0;
        if (!append(h, used, "t%u.txt=\"%lu.%03luV\"", fields[ch][2],
                    (unsigned long)(r->vpp_mv / 1000u), (unsigned long)(r->vpp_mv % 1000u))) return 0;
    }
    if (!status_command(h, used)) return 0;
    /* Existing HMI labels/buttons overlap the waveform rectangles. */
    for (ch = 0u; ch < sizeof(overlays)/sizeof(overlays[0]); ++ch)
        if (!append(h, used, "ref %s", overlays[ch])) return 0;
    return 1;
}
void HmiProtocol_Init(HmiProtocol *h, HmiSend send, void *context, uint32_t now)
{
    memset(h, 0, sizeof(*h));
    h->send = send; h->context = context; h->page = 255u;
    h->last_query = now - 1000u;
}
static void packet(HmiProtocol *h)
{
    const uint8_t *a = h->rx;
    unsigned n = h->rx_size;
    if (n == 7u && a[0] == 0x65u && a[1] == SCOPE_HMI_PAGE && a[3] == 0u) {
        h->page = SCOPE_HMI_PAGE;
        if (a[2] == SCOPE_HMI_AUTO_ID) {
            h->paused = 0u;
            h->events |= HMI_EVENT_AUTO;
            h->status_dirty = 1u;
        } else if (a[2] == SCOPE_HMI_STOP_ID) {
            h->paused ^= 1u;
            h->events |= HMI_EVENT_STOP;
            h->status_dirty = 1u;
        }
    } else if (n == 5u && a[0] == 0x66u) {
        if (h->page != a[1]) { h->events |= HMI_EVENT_PAGE; h->status_dirty = 1u; }
        h->page = a[1];
    } else if (n == 4u && a[0] == 0xfeu) ++h->ready_count;
    else if (n == 4u && a[0] == 0xfdu) ++h->done_count;
    else if (n == 4u && a[0] == 0x88u) {
        h->page = 255u;
        h->events |= HMI_EVENT_PAGE;
    }
}
void HmiProtocol_Receive(HmiProtocol *h, uint8_t byte)
{
    unsigned n;
    if (h->rx_size >= sizeof(h->rx)) {
        h->rx_size = 0u;
        ++h->errors;
    }
    h->rx[h->rx_size++] = byte;
    n = h->rx_size;
    if (n >= 3u && h->rx[n-1u] == 255u && h->rx[n-2u] == 255u && h->rx[n-3u] == 255u) {
        packet(h);
        h->rx_size = 0u;
    }
}
int HmiProtocol_CanSubmit(const HmiProtocol *h)
{
    return h->state == IDLE && !h->transmitting && !h->paused && h->page == SCOPE_HMI_PAGE;
}
int HmiProtocol_Submit(HmiProtocol *h, const HmiSnapshot *snapshot)
{
    if (!HmiProtocol_CanSubmit(h)) return 0;
    h->snapshot = *snapshot;
    h->channel = h->failed = 0u;
    h->state = CLEAR;
    return 1;
}
void HmiProtocol_TxComplete(HmiProtocol *h) { h->transmitting = 0u; }
void HmiProtocol_TransportError(HmiProtocol *h)
{
    h->transmitting = 0u;
    ++h->errors;
    /* A transport failure may leave the display waiting for raw bytes. */
    if (h->state == WAIT_READY || h->state == WAIT_DONE) {
        h->failed = 1u;
        h->state = DATA;
    } else h->state = RESYNC;
}
uint8_t HmiProtocol_TakeEvents(HmiProtocol *h)
{
    uint8_t events = h->events;
    h->events = 0u;
    return events;
}
void HmiProtocol_Process(HmiProtocol *h, uint32_t now)
{
    unsigned n = 0u;
    unsigned id = h->channel == 0u ? SCOPE_HMI_WAVE_A_ID : SCOPE_HMI_WAVE_B_ID;
    unsigned width = h->channel == 0u ? SCOPE_WAVE_A_WIDTH : SCOPE_WAVE_B_WIDTH;
    if (h->transmitting) return;
    switch (h->state) {
    case IDLE:
        if (h->status_dirty && h->page == SCOPE_HMI_PAGE) {
            if (status_command(h, &n) && transmit(h, h->command, (uint16_t)n)) h->status_dirty = 0u;
        } else if ((uint32_t)(now - h->last_query) >= 1000u) {
            if (append(h, &n, "sendme") && transmit(h, h->command, (uint16_t)n)) h->last_query = now;
        }
        break;
    case CLEAR:
        if (h->page != SCOPE_HMI_PAGE) { h->state = IDLE; break; }
        if (append(h, &n, "cle %u,0", id) && transmit(h, h->command, (uint16_t)n)) h->state = ADDT;
        break;
    case ADDT:
        h->ready_before = h->ready_count;
        if (append(h, &n, "addt %u,0,%u", id, width) && transmit(h, h->command, (uint16_t)n)) {
            h->state = WAIT_READY; h->deadline = now + 500u;
        }
        break;
    case WAIT_READY:
        if (h->ready_count != h->ready_before) h->state = DATA;
        else if ((int32_t)(now - h->deadline) >= 0) {
            ++h->errors; h->failed = 1u; h->state = DATA;
        }
        break;
    case DATA:
        h->done_before = h->done_count;
        if (transmit(h, h->channel == 0u ? h->snapshot.a : h->snapshot.b, (uint16_t)width)) {
            h->state = WAIT_DONE; h->deadline = now + 500u;
        }
        break;
    case WAIT_DONE:
        if (h->done_count != h->done_before) {
            if (h->failed) h->state = RESYNC;
            else if (h->channel == 0u) { h->channel = 1u; h->state = CLEAR; }
            else h->state = VALUES;
        } else if ((int32_t)(now - h->deadline) >= 0) {
            ++h->errors; h->state = RESYNC;
        }
        break;
    case VALUES:
        if (h->page != SCOPE_HMI_PAGE) { h->state = IDLE; break; }
        if (values_command(h, &n) && transmit(h, h->command, (uint16_t)n)) {
            h->status_dirty = 0u; h->state = IDLE;
        }
        break;
    case RESYNC:
        memset(h->command, 255, 3u); n = 3u;
        if (append(h, &n, "sendme") && transmit(h, h->command, (uint16_t)n)) {
            h->page = 255u; h->last_query = now; h->state = IDLE;
        }
        break;
    default: h->state = RESYNC; break;
    }
}
