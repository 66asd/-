#include "scope_app.h"
#include "scope_acquisition.h"
#include "measure_frequency.h"
#include "measure_rms.h"
#include "measure_vpp.h"
#include "sine_fit.h"
#include "scope_waveform.h"
#include "hmi_uart.h"
#include "main.h"
#include <string.h>
#include <math.h>

typedef struct {
    FrequencyMeter frequency;
    RmsMeter rms;
    VppMeter vpp;
    SineFit fit;
    uint16_t trace[SCOPE_TRACE_SAMPLES];
    uint16_t trace_count, stride, countdown;
    uint8_t ready, fitting, pixels_valid, model_valid;
    float hz, mean, ac_rms, peak_to_peak, model_vpp, model_hz, fit_hz;
    uint8_t frequency_valid;
} ScopeChannel;
static ScopeChannel channels[2];
static uint32_t raw_copy[SCOPE_DMA_HALF_WORDS], window_count, last_display;
static HmiSnapshot display;
volatile ScopeDiagnostics g_scope_diagnostics;

static void reset_channel(ScopeChannel *c)
{
    memset(c, 0, sizeof(*c));
    Frequency_Init(&c->frequency);
    c->stride = 1u;
    c->mean = 2048.0f;
}
static void finish_window(ScopeChannel *c)
{
    RmsResult rms = Rms_Result(&c->rms);
    float robust = Vpp_RobustCounts(&c->vpp);
    float hz = Frequency_FinishWindow(&c->frequency, SCOPE_SAMPLE_RATE_HZ);
    c->mean = rms.mean_counts;
    c->ac_rms = rms.ac_counts;
    c->hz = hz;
    c->frequency_valid = (uint8_t)(hz >= SCOPE_MIN_FREQUENCY_HZ * 0.98f
        && hz <= SCOPE_MAX_FREQUENCY_HZ * 1.01f && robust >= 20.0f);
    if (!c->frequency_valid || fabsf(hz - c->model_hz) > hz * 0.002f) c->model_valid = 0u;
    c->peak_to_peak = c->model_valid ? c->model_vpp : robust;
    Frequency_SetThreshold(&c->frequency, (uint16_t)(c->mean + 0.5f), (uint16_t)robust);
    Rms_Reset(&c->rms);
    Vpp_Reset(&c->vpp);
}
static void push_sample(ScopeChannel *c, uint16_t sample, uint32_t index)
{
    int crossing = Frequency_Push(&c->frequency, sample, index);
    Rms_Push(&c->rms, sample);
    if (crossing) Rms_MarkCycle(&c->rms);
    Vpp_Push(&c->vpp, sample);
    if (!c->ready) {
        if (c->countdown == 0u) {
            if (c->trace_count == 0u) {
                c->stride = Waveform_Stride(c->frequency_valid ? c->hz : 0.0f,
                                            SCOPE_SAMPLE_RATE_HZ, SCOPE_TRACE_SAMPLES);
            }
            c->trace[c->trace_count++] = sample;
            c->countdown = c->stride;
            if (c->trace_count == SCOPE_TRACE_SAMPLES) c->ready = 1u;
        }
        --c->countdown;
    }
}
static void service_trace(unsigned ch)
{
    ScopeChannel *c = &channels[ch];
    uint8_t *pixels = ch == 0u ? display.a : display.b;
    uint16_t width = ch == 0u ? SCOPE_WAVE_A_WIDTH : SCOPE_WAVE_B_WIDTH;
    uint16_t height = ch == 0u ? SCOPE_WAVE_A_HEIGHT : SCOPE_WAVE_B_HEIGHT;
    float rate = (float)SCOPE_SAMPLE_RATE_HZ / c->stride;
    if (!c->ready) return;
    if (!c->fitting) {
        c->fit_hz = c->frequency_valid ? c->hz : 0.0f;
        SineFit_Start(&c->fit, c->trace, c->trace_count, rate, c->fit_hz);
        c->fitting = 1u;
    }
    if (!SineFit_Step(&c->fit, 24u)) return;
    c->model_valid = (uint8_t)(c->fit.valid && c->frequency_valid
        && fabsf(c->hz - c->fit_hz) <= c->hz * 0.002f);
    if (c->model_valid) {
        c->model_vpp = c->fit.amplitude * 2.0f;
        c->model_hz = c->fit_hz;
        c->peak_to_peak = c->model_vpp;
    }
    c->pixels_valid = (uint8_t)Waveform_Render(c->trace, c->trace_count, rate,
        c->frequency_valid ? c->hz : 0.0f, c->model_valid ? c->fit.dc : c->mean,
        c->peak_to_peak, pixels, width, height);
    c->trace_count = 0u;
    c->countdown = c->ready = c->fitting = 0u;
}
int ScopeApp_Init(void)
{
    memset((void *)&g_scope_diagnostics, 0, sizeof(g_scope_diagnostics));
    reset_channel(&channels[0]); reset_channel(&channels[1]);
    memset(&display, 0, sizeof(display));
    window_count = 0u;
    last_display = HAL_GetTick();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    if (!HmiUart_Init()) return 0;
    return Acquisition_Start();
}
void ScopeApp_Process(void)
{
    uint32_t began = DWT->CYCCNT, first_index, now, i;
    int gap = 0, available;
    HmiProtocol *h = HmiUart_Protocol();
    uint8_t events;
    HmiUart_Process();
    events = HmiProtocol_TakeEvents(h);
    if ((events & (HMI_EVENT_AUTO | HMI_EVENT_PAGE)) != 0u) {
        channels[0].pixels_valid = channels[1].pixels_valid = 0u;
        last_display = HAL_GetTick() - SCOPE_DISPLAY_INTERVAL_MS;
    }
    available = Acquisition_CopyLatest(raw_copy, &first_index, &gap);
    if (available < 0 || gap) {
        reset_channel(&channels[0]); reset_channel(&channels[1]);
        window_count = 0u;
    }
    if (available > 0) {
        for (i = 0u; i < SCOPE_DMA_HALF_WORDS; ++i) {
            push_sample(&channels[0], (uint16_t)(raw_copy[i] & 4095u), first_index + i);
            push_sample(&channels[1], (uint16_t)((raw_copy[i] >> 16) & 4095u), first_index + i);
            if (++window_count == SCOPE_MEASURE_SAMPLES) {
                finish_window(&channels[0]); finish_window(&channels[1]);
                window_count = 0u;
            }
        }
        ++g_scope_diagnostics.processed_blocks;
    }
    service_trace(0u);
    service_trace(1u);
    now = HAL_GetTick();
    if (HmiProtocol_CanSubmit(h) && channels[0].pixels_valid && channels[1].pixels_valid
        && (uint32_t)(now - last_display) >= SCOPE_DISPLAY_INTERVAL_MS) {
        for (i = 0u; i < 2u; ++i) {
            ScopeChannel *c = &channels[i];
            display.reading[i].frequency_tenths = (uint32_t)(c->hz * 10.0f + 0.5f);
            display.reading[i].rms_mv = (uint32_t)(c->ac_rms * SCOPE_REFERENCE_MV / SCOPE_ADC_FULL_SCALE + 0.5f);
            display.reading[i].vpp_mv = (uint32_t)(c->peak_to_peak * SCOPE_REFERENCE_MV / SCOPE_ADC_FULL_SCALE + 0.5f);
            display.reading[i].frequency_valid = c->frequency_valid;
        }
        if (HmiProtocol_Submit(h, &display)) {
            last_display = now;
            ++g_scope_diagnostics.displayed_frames;
        }
    }
    g_scope_diagnostics.dma_overruns = Acquisition_Overruns();
    g_scope_diagnostics.hmi_errors = h->errors;
    g_scope_diagnostics.adc_fault = (uint8_t)Acquisition_HasFault();
    if (g_scope_diagnostics.adc_fault) Error_Handler();
    i = DWT->CYCCNT - began;
    if (i > g_scope_diagnostics.max_process_cycles) g_scope_diagnostics.max_process_cycles = i;
}
