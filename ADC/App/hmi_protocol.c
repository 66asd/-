#include "hmi_protocol.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
enum { IDLE, VALUES, CLEAR, DRAW, REFRESH, RESYNC };

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
    h->tx_started = h->now;
    if (h->send(h->context, data, size)) { h->tx_bytes += size; return 1; }
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
    return status_command(h, used);
}

static int overlays_command(HmiProtocol *h, unsigned *used)
{
    static const char * const names[]={"t0","t2","t4","t6","t8","t10","b0","b1","b2"};
    unsigned i;
    for(i=0u;i<sizeof(names)/sizeof(names[0]);++i)
        if(!append(h,used,"ref %s",names[i])) return 0;
    return 1;
}
void HmiProtocol_Init(HmiProtocol *h, HmiSend send, void *context, uint32_t now)
{
    memset(h,0,sizeof(*h)); h->send=send; h->context=context;
    h->page=255u; h->now=now; h->last_query=now;
}
static void packet(HmiProtocol *h)
{
    const uint8_t *a=h->rx;
    unsigned n=h->rx_size;
    if(n==7u && a[0]==0x65u && a[1]==SCOPE_HMI_PAGE && a[3]==0u) {
        h->page=SCOPE_HMI_PAGE;
        if(a[2]==SCOPE_HMI_AUTO_ID) { h->paused=0u; h->events|=HMI_EVENT_AUTO; h->status_dirty=1u; }
        else if(a[2]==SCOPE_HMI_STOP_ID) { h->paused^=1u; h->events|=HMI_EVENT_STOP; h->status_dirty=1u; }
    } else if(n==5u && a[0]==0x66u) {
        if(h->page!=a[1]) { h->events|=HMI_EVENT_PAGE; h->status_dirty=1u; }
        h->page=a[1];
    } else if(n==4u && a[0]==0x88u) {
        h->page=255u; h->boot_sent=0u; h->events|=HMI_EVENT_PAGE;
    }
}
void HmiProtocol_Receive(HmiProtocol *h,uint8_t byte)
{
    unsigned n;
    ++h->rx_bytes;
    if(h->rx_size>=sizeof(h->rx)) { h->rx_size=0u; ++h->errors; }
    h->rx[h->rx_size++]=byte; n=h->rx_size;
    if(n>=3u && h->rx[n-1u]==255u && h->rx[n-2u]==255u && h->rx[n-3u]==255u) {
        packet(h); h->rx_size=0u;
    }
}
int HmiProtocol_CanSubmit(const HmiProtocol *h)
{
    return h->boot_sent && h->state==IDLE && !h->transmitting && !h->paused && h->page==SCOPE_HMI_PAGE;
}
int HmiProtocol_Submit(HmiProtocol *h,const HmiSnapshot *snapshot)
{
    if(!HmiProtocol_CanSubmit(h)) return 0;
    h->snapshot=*snapshot; h->cursor=0u; h->state=VALUES; return 1;
}
void HmiProtocol_TxComplete(HmiProtocol *h) { h->transmitting=0u; }
void HmiProtocol_TransportError(HmiProtocol *h)
{
    h->transmitting=0u; ++h->errors; h->state=RESYNC;
}
uint8_t HmiProtocol_TakeEvents(HmiProtocol *h)
{
    uint8_t e=h->events; h->events=0u; return e;
}
void HmiProtocol_Process(HmiProtocol *h,uint32_t now)
{
    unsigned n=0u, i;
    h->now=now;
    if(h->transmitting) return;
    if((h->paused || h->page!=SCOPE_HMI_PAGE) && h->state!=IDLE && h->state!=RESYNC)
        h->state=REFRESH;
    switch(h->state) {
    case IDLE:
        /* Enter the requested measurement page once; output does not depend on RX acknowledgements. */
        if(!h->boot_sent && (uint32_t)(now-h->last_query)>=800u) {
            if(append(h,&n,"page 3") && append(h,&n,"sendme") && transmit(h,h->command,(uint16_t)n)) {
                h->boot_sent=1u; h->page=SCOPE_HMI_PAGE; h->last_query=now;
            }
        } else if(h->status_dirty && h->page==SCOPE_HMI_PAGE) {
            if(status_command(h,&n) && transmit(h,h->command,(uint16_t)n)) h->status_dirty=0u;
        } else if(h->boot_sent && (uint32_t)(now-h->last_query)>=1000u) {
            if(append(h,&n,"sendme") && transmit(h,h->command,(uint16_t)n)) h->last_query=now;
        }
        break;
    case VALUES:
        if(values_command(h,&n) && transmit(h,h->command,(uint16_t)n)) h->state=CLEAR;
        break;
    case CLEAR:
        if(append(h,&n,"cle %u,0",SCOPE_HMI_WAVE_A_ID) && append(h,&n,"cle %u,0",SCOPE_HMI_WAVE_B_ID)
            && overlays_command(h,&n)
            && transmit(h,h->command,(uint16_t)n)) h->state=DRAW;
        break;
    case DRAW:
        i=h->cursor;
        /* Four point pairs per packet: bounded stop latency, no transparent-mode handshake. */
        while(i<h->cursor+4u && i<SCOPE_WAVE_A_WIDTH) {
            if(!append(h,&n,"add %u,0,%u",SCOPE_HMI_WAVE_A_ID,h->snapshot.a[i])) return;
            if(i<SCOPE_WAVE_B_WIDTH && !append(h,&n,"add %u,0,%u",SCOPE_HMI_WAVE_B_ID,h->snapshot.b[i])) return;
            ++i;
        }
        if(transmit(h,h->command,(uint16_t)n)) {
            h->cursor=(uint16_t)i;
            if(i==SCOPE_WAVE_A_WIDTH) h->state=REFRESH;
        }
        break;
    case REFRESH:
        if(h->page!=SCOPE_HMI_PAGE) h->state=IDLE;
        else if(overlays_command(h,&n) && transmit(h,h->command,(uint16_t)n)) h->state=IDLE;
        break;
    case RESYNC:
        memset(h->command,255,3u); n=3u;
        if(append(h,&n,"page 3") && append(h,&n,"sendme") && transmit(h,h->command,(uint16_t)n)) {
            h->page=SCOPE_HMI_PAGE; h->boot_sent=1u; h->last_query=now; h->state=IDLE;
        }
        break;
    default: h->state=RESYNC; break;
    }
}
