#include "vofa_raw.h"
#include "scope_config.h"
#include <string.h>

enum { PACKET_SAMPLES = 8, FRAME_BYTES = 20 };
static uint16_t raw[SCOPE_DMA_HALF_WORDS], count, cursor;
static uint8_t packet[PACKET_SAMPLES * FRAME_BYTES], synchronize;
static volatile uint8_t transmitting;
static uint32_t command, tx_started, rate, segment;
static VofaSend port_send;
static void *port_context;
volatile VofaDiagnostics g_vofa_diagnostics;

void VofaRaw_Init(VofaSend send, void *context)
{
    port_send=send; port_context=context; command=segment=0u;
    memset((void *)&g_vofa_diagnostics,0,sizeof(g_vofa_diagnostics));
    VofaRaw_Enable(0);
}
void VofaRaw_Enable(int enabled)
{
    /* Caller must abort any in-flight UART transmission before changing modes. */
    g_vofa_diagnostics.enabled=(uint8_t)(enabled!=0);
    count=cursor=0u; transmitting=0u; synchronize=1u;
}
int VofaRaw_Enabled(void) { return g_vofa_diagnostics.enabled!=0u; }
int VofaRaw_Command(uint8_t byte)
{
    command=(command<<8)|byte;
    if(command==0x564F4641u) { command=0u; return 1; } /* VOFA */
    if(command==0x484D4921u) { command=0u; return 2; } /* HMI! */
    return 0;
}
void VofaRaw_Offer(const uint32_t *packed, uint16_t size, uint32_t sample_rate)
{
    unsigned i;
    if(!VofaRaw_Enabled() || count || synchronize || transmitting || !sample_rate ||
       size!=SCOPE_DMA_HALF_WORDS) return;
    /* One already-validated DMA half is contiguous. No decimation/filtering/calibration. */
    for(i=0u;i<size;++i) raw[i]=(uint16_t)(packed[i]&4095u);
    rate=sample_rate; count=size; cursor=0u;
    segment=(segment+1u)&0x00FFFFFFu;
    g_vofa_diagnostics.segment_id=segment;
    g_vofa_diagnostics.sample_rate_hz=rate;
}
void VofaRaw_Process(uint32_t now)
{
    static const uint8_t tail[4]={0x00,0x00,0x80,0x7F};
    unsigned i, n;
    uint16_t bytes;
    if(!VofaRaw_Enabled() || transmitting) return;
    if(count && cursor==count) {
        g_vofa_diagnostics.samples_sent+=count;
        ++g_vofa_diagnostics.segments_sent;
        count=cursor=0u;
    }
    if(synchronize) {
        memcpy(packet,tail,4u); bytes=4u; n=0u;
    } else {
        if(!count) return;
        n=count-cursor;
        if(n>PACKET_SAMPLES) n=PACKET_SAMPLES;
        for(i=0u;i<n;++i) {
            /* JustFloat: ADC1 code, hardware Fs, segment ID, index within segment. */
            float values[4];
            values[0]=(float)raw[cursor+i]; values[1]=(float)rate;
            values[2]=(float)segment; values[3]=(float)(cursor+i);
            memcpy(packet+i*FRAME_BYTES,values,16u);
            memcpy(packet+i*FRAME_BYTES+16u,tail,4u);
        }
        bytes=(uint16_t)(n*FRAME_BYTES);
    }
    transmitting=1u; tx_started=now;
    if(port_send(port_context,packet,bytes)) {
        synchronize=0u; cursor=(uint16_t)(cursor+n);
    } else transmitting=0u;
}
void VofaRaw_TxComplete(void) { transmitting=0u; }
int VofaRaw_TimedOut(uint32_t now)
{
    return transmitting && (uint32_t)(now-tx_started)>200u;
}
void VofaRaw_TransportError(void)
{
    transmitting=0u; count=cursor=0u; synchronize=1u;
    ++g_vofa_diagnostics.errors;
}
