#include "scope_app.h"
#include "scope_acquisition.h"
#include "measure_frequency.h"
#include "measure_rms.h"
#include "measure_vpp.h"
#include "sine_fit.h"
#include "scope_waveform.h"
#include "sample_rate.h"
#include "hmi_uart.h"
#include "vofa_raw.h"
#include "main.h"
#include <string.h>

enum { CAPTURE, MOMENTS, PERIODS, FIT };
typedef struct {
    FrequencyMeter frequency;
    RmsMeter rms;
    VppMeter vpp;
    SineFit fit;
    uint16_t trace[SCOPE_TRACE_SAMPLES];
    uint16_t count, work, stride, countdown, search_stride;
    uint8_t state, active, valid, frame_probe;
    uint16_t low, high;
    uint32_t good_at;
    float hz, mean, ac_rms, peak_to_peak, frame_rate, measured_hz;
    float frame_mean, frame_rms, frame_vpp;
} ScopeChannel;
static ScopeChannel channels[2];
static uint32_t raw_copy[SCOPE_DMA_HALF_WORDS], last_display, last_probe;
static SampleRatePolicy rate_policy;
static HmiSnapshot display;
static uint8_t probing, probe_done;
volatile ScopeDiagnostics g_scope_diagnostics;

static void reset_capture(ScopeChannel *c)
{
    c->state = CAPTURE; c->count = c->work = c->countdown = 0u;
    c->low = 4095u; c->high = 0u;
}
static int change_rate(uint32_t rate)
{
    unsigned ch;
    if (rate != Acquisition_Rate() && !Acquisition_SetRate(rate)) {
        g_scope_diagnostics.adc_fault = 1u; return 0;
    }
    for (ch = 0u; ch < 2u; ++ch) reset_capture(&channels[ch]);
    ++g_scope_diagnostics.rate_changes;
    return 1;
}
static void collect(ScopeChannel *c, unsigned ch)
{
    uint32_t i, fs = Acquisition_Rate();
    if (c->state != CAPTURE || (probing && (probe_done & (1u << ch)))) return;
    if (c->count == 0u) {
        uint16_t max_stride=(uint16_t)(fs/5000u);
        c->stride = probing ? 1u : c->valid ? SampleRate_Stride(fs,c->hz) : c->search_stride;
        if(max_stride<1u) max_stride=1u;
        if(c->stride>max_stride) c->stride=max_stride;
        if (c->stride < 1u) c->stride = 1u;
        c->frame_rate = (float)fs / c->stride;
        c->frame_probe = probing;
    }
    /* Capture selected samples only; analyze the immutable frame cooperatively. */
    for (i = c->countdown; i < SCOPE_DMA_HALF_WORDS; i += c->stride) {
        uint16_t x = (uint16_t)((raw_copy[i] >> (ch * 16u)) & 4095u);
        c->trace[c->count++] = x;
        if (x < c->low) c->low = x;
        if (x > c->high) c->high = x;
        if (c->count == SCOPE_TRACE_SAMPLES) {
            c->state = MOMENTS; c->work = 0u;
            Rms_Reset(&c->rms); Vpp_Reset(&c->vpp); return;
        }
    }
    c->countdown = (uint16_t)(i - SCOPE_DMA_HALF_WORDS);
}
static void finish_trace(ScopeChannel *c, unsigned ch, uint32_t now)
{
    uint8_t *pixels = ch == 0u ? display.a : display.b;
    unsigned width = ch == 0u ? SCOPE_WAVE_A_WIDTH : SCOPE_WAVE_B_WIDTH;
    unsigned height = ch == 0u ? SCOPE_WAVE_A_HEIGHT : SCOPE_WAVE_B_HEIGHT;
    float vpp = c->frame_vpp, mean = c->frame_mean;
    uint8_t used_fit=0u;
    if (c->measured_hz > 0.0f) {
        vpp=Vpp_SelectCounts(c->frame_vpp,c->fit.amplitude*2.0f,c->fit.valid,
                              c->measured_hz,c->frame_rate,&used_fit);
        if (used_fit) mean=c->fit.dc;
        c->good_at = now;
    }
    if (!(c->frame_probe && c->valid && c->measured_hz==0.0f)) {
        g_scope_diagnostics.raw_min[ch]=c->low;
        g_scope_diagnostics.raw_max[ch]=c->high;
        g_scope_diagnostics.vpp_raw_counts[ch]=c->frame_vpp;
        g_scope_diagnostics.vpp_fit_counts[ch]=c->fit.amplitude*2.0f;
        g_scope_diagnostics.vpp_used_fit[ch]=used_fit;
        c->mean=mean; c->ac_rms=c->frame_rms; c->peak_to_peak=vpp;
        (void)Waveform_Render(c->trace,c->count,c->frame_rate,c->measured_hz,
                              mean,vpp,pixels,(uint16_t)width,(uint16_t)height);
    }
    if (c->frame_probe) probe_done |= (uint8_t)(1u << ch);
    ++g_scope_diagnostics.analyzed_frames;
    reset_capture(c);
}
static void analyze(ScopeChannel *c, unsigned ch, uint32_t now)
{
    uint16_t budget = 32u;
    if (c->state == CAPTURE) return;
    if (c->state == MOMENTS) {
        while (budget-- && c->work < c->count) {
            uint16_t x = c->trace[c->work++];
            Rms_Push(&c->rms,x); Vpp_Push(&c->vpp,x);
        }
        if (c->work == c->count) {
            RmsResult r = Rms_Result(&c->rms);
            c->frame_mean = r.mean_counts; c->frame_rms = r.ac_counts;
            c->frame_vpp = Vpp_RobustCounts(&c->vpp);
            c->active = (uint8_t)(c->high - c->low >= 20u);
            Frequency_Init(&c->frequency);
            Frequency_SetThreshold(&c->frequency,(uint16_t)(c->frame_mean+0.5f),(uint16_t)c->frame_vpp);
            Rms_Reset(&c->rms); c->work = 0u; c->state = PERIODS;
        }
    } else if (c->state == PERIODS) {
        while (budget-- && c->work < c->count) {
            uint16_t x = c->trace[c->work];
            int crossing = Frequency_Push(&c->frequency,x,c->work++);
            Rms_Push(&c->rms,x);
            if (crossing) Rms_MarkCycle(&c->rms);
        }
        if (c->work == c->count) {
            float hz = Frequency_FinishWindow(&c->frequency,c->frame_rate);
            uint16_t max_stride = (uint16_t)(Acquisition_Rate()/5000u);
            RmsResult r = Rms_Result(&c->rms);
            c->frame_rms = r.ac_counts;
            c->measured_hz = (c->active && hz >= SCOPE_MIN_FREQUENCY_HZ * 0.9f
                              && hz < c->frame_rate * 0.49f) ? hz : 0.0f;
            if (c->measured_hz > 0.0f) {
                c->hz = hz; c->valid = 1u; c->search_stride = 1u;
            } else if (!c->active || (!c->frame_probe && (uint32_t)(now-c->good_at)>600u)) {
                c->valid = 0u;
            }
            if (!c->valid) {
                unsigned next = c->search_stride * 10u;
                if (max_stride < 1u) max_stride = 1u;
                c->search_stride = (uint16_t)(next > max_stride ? max_stride : next);
            }
            SineFit_Start(&c->fit,c->trace,c->count,c->frame_rate,c->measured_hz);
            c->state = FIT;
        }
    } else if (SineFit_Step(&c->fit,24u)) finish_trace(c,ch,now);
}
int ScopeApp_Init(void)
{
    unsigned ch;
    memset((void *)&g_scope_diagnostics,0,sizeof(g_scope_diagnostics));
    memset(&display,0,sizeof(display)); memset(channels,0,sizeof(channels));
    for (ch=0u; ch<2u; ++ch) {
        channels[ch].search_stride=1u; channels[ch].mean=2048.0f;
        reset_capture(&channels[ch]);
    }
    memset(display.a,SCOPE_WAVE_A_HEIGHT/2u,sizeof(display.a));
    memset(display.b,SCOPE_WAVE_B_HEIGHT/2u,sizeof(display.b));
    memset(&rate_policy,0,sizeof(rate_policy));
    probing=1u; probe_done=0u; last_probe=last_display=HAL_GetTick();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT=0u; DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    if (!HmiUart_Init()) return 0;
    /* A failed ADC must not silence UART or freeze the entire UI. */
    if (!Acquisition_Start()) g_scope_diagnostics.adc_fault=1u;
    return 1;
}
void ScopeApp_Process(void)
{
    uint32_t began=DWT->CYCCNT, index=0u, now=HAL_GetTick(), i;
    int gap=0, available;
    HmiProtocol *h=HmiUart_Protocol();
    uint8_t events;
    HmiUart_Process(); events=HmiProtocol_TakeEvents(h);
    if (!g_scope_diagnostics.adc_fault) {
        available=Acquisition_CopyLatest(raw_copy,&index,&gap);
        if (available<0 || gap) {
            for(i=0u;i<2u;++i) if(channels[i].state==CAPTURE) reset_capture(&channels[i]);
        }
        if(available>0) {
            VofaRaw_Offer(raw_copy,SCOPE_DMA_HALF_WORDS,Acquisition_Rate());
            collect(&channels[0],0u); collect(&channels[1],1u); ++g_scope_diagnostics.processed_blocks;
        }
        analyze(&channels[0],0u,now); analyze(&channels[1],1u,now);
        if(probing && probe_done==3u) { probing=0u; last_probe=now; }
        if(!probing && ((events&HMI_EVENT_AUTO) || (uint32_t)(now-last_probe)>=SCOPE_PROBE_INTERVAL_MS)) {
            probing=1u; probe_done=0u; (void)change_rate(SCOPE_SAMPLE_RATE_HZ);
        } else if(!probing) {
            uint32_t target=SampleRate_Decide(&rate_policy,Acquisition_Rate(),
                channels[0].valid?channels[0].hz:0.0f,channels[1].valid?channels[1].hz:0.0f,
                (channels[0].active&&!channels[0].valid)||(channels[1].active&&!channels[1].valid),now);
            if(target!=Acquisition_Rate()) (void)change_rate(target);
        }
        if(Acquisition_HasFault()) g_scope_diagnostics.adc_fault=1u;
    }
    if(!VofaRaw_Enabled() && HmiProtocol_CanSubmit(h) && (uint32_t)(now-last_display)>=SCOPE_DISPLAY_INTERVAL_MS) {
        for(i=0u;i<2u;++i) {
            ScopeChannel *c=&channels[i];
            display.reading[i].frequency_tenths=(uint32_t)(c->hz*10.0f+0.5f);
            display.reading[i].rms_mv=(uint32_t)(c->ac_rms*SCOPE_REFERENCE_MV/SCOPE_ADC_FULL_SCALE+0.5f);
            display.reading[i].vpp_mv=(uint32_t)(c->peak_to_peak*SCOPE_REFERENCE_MV/SCOPE_ADC_FULL_SCALE+0.5f);
            display.reading[i].frequency_valid=(uint8_t)(c->valid&&!g_scope_diagnostics.adc_fault);
        }
        if(HmiProtocol_Submit(h,&display)) { last_display=now; ++g_scope_diagnostics.displayed_frames; }
    }
    g_scope_diagnostics.sample_rate_hz=Acquisition_Rate();
    g_scope_diagnostics.dma_overruns=Acquisition_Overruns();
    g_scope_diagnostics.hmi_errors=h->errors; g_scope_diagnostics.hmi_page=h->page;
    g_scope_diagnostics.uart_tx_bytes=h->tx_bytes; g_scope_diagnostics.uart_rx_bytes=h->rx_bytes;
    for(i=0u;i<2u;++i) g_scope_diagnostics.analysis_rate_hz[i]=(uint32_t)channels[i].frame_rate;
    i=DWT->CYCCNT-began;
    if(i>g_scope_diagnostics.max_process_cycles) g_scope_diagnostics.max_process_cycles=i;
}
