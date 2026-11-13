#include "measure_frequency.h"
#include "measure_rms.h"
#include "measure_vpp.h"
#include "sine_fit.h"
#include "scope_waveform.h"
#include "hmi_protocol.h"
#include "scope_app.h"
#include "scope_acquisition.h"
#include "hmi_uart.h"
#include "main.h"
#include <math.h>
#include <string.h>

volatile uint32_t test_failed_line, test_case, test_passed;
volatile float test_details[8];
#ifdef __CC_ARM
#define TEST_BREAK() __breakpoint(0)
#else
#define TEST_BREAK() __asm volatile("bkpt #0")
#endif
#define CHECK(x) do { if (!(x)) { test_failed_line = __LINE__; TEST_BREAK(); for (;;) {} } } while (0)
static uint16_t samples[SCOPE_TRACE_SAMPLES];
static uint8_t pixels[SCOPE_WAVE_A_WIDTH];
static FrequencyMeter frequency;
static RmsMeter rms;
static VppMeter vpp;
static SineFit fit;
static uint32_t random_state = 1234567u;
static float noise(void)
{
    random_state = random_state * 1664525u + 1013904223u;
    return (float)((int32_t)(random_state >> 16) - 32768) / 32768.0f;
}
static uint16_t sine_sample(float hz, uint32_t index, float phase, float dc, int noisy)
{
    float x = dc + 1200.0f * sinf(6.28318530718f * hz * ((float)index / 200000.0f) + phase);
    if (noisy) x += 3.0f * noise();
    if (x < 0.0f) x = 0.0f;
    if (x > 4095.0f) x = 4095.0f;
    return (uint16_t)(x + 0.5f);
}
static void check_sine(float hz, float phase, float dc)
{
    uint32_t i;
    uint16_t stride = Waveform_Stride(hz, SCOPE_SAMPLE_RATE_HZ, SCOPE_TRACE_SAMPLES);
    float measured, robust;
    RmsResult result;
    Frequency_Init(&frequency); Rms_Reset(&rms); Vpp_Reset(&vpp);
    Frequency_SetThreshold(&frequency, (uint16_t)dc, 2400u);
    for (i = 0u; i < 10000u; ++i) {
        uint16_t x = sine_sample(hz, i, phase, dc, 1);
        if (i == 4567u || i == 6789u) x = 4095u;
        int crossing = Frequency_Push(&frequency, x, i);
        Rms_Push(&rms, x);
        if (crossing) Rms_MarkCycle(&rms);
        if (i == 4567u || i == 6789u) x = 4095u;
        Vpp_Push(&vpp, x);
    }
    measured = Frequency_FinishWindow(&frequency, SCOPE_SAMPLE_RATE_HZ);
    result = Rms_Result(&rms);
    robust = Vpp_RobustCounts(&vpp);
    test_details[0] = hz; test_details[1] = measured;
    test_details[2] = result.ac_counts; test_details[3] = robust;
    CHECK(fabsf(measured - hz) < hz * 0.002f);
    CHECK(fabsf(result.ac_counts - 848.5281f) < 6.0f);
    CHECK(fabsf(result.mean_counts - dc) < 5.0f);
    CHECK(result.total_counts > result.ac_counts + 100.0f);
    CHECK(robust < 2430.0f && robust > 2100.0f);
    for (i = 0u; i < SCOPE_TRACE_SAMPLES; ++i)
        samples[i] = sine_sample(hz, i * stride, phase, dc, 1);
    samples[601] = 4095u;
    SineFit_Start(&fit, samples, SCOPE_TRACE_SAMPLES, 200000.0f / stride, measured);
    for (i = 0u; i < 200u && !SineFit_Step(&fit, 24u); ++i) {}
    test_details[4] = fit.amplitude; test_details[5] = fit.residual_ratio;
    CHECK(fit.done && fit.valid);
    CHECK(fabsf(2.0f * fit.amplitude - 2400.0f) < 20.0f);
    CHECK(Waveform_Render(samples, SCOPE_TRACE_SAMPLES, 200000.0f / stride,
                         measured, fit.dc, 2.0f * fit.amplitude,
                         pixels, SCOPE_WAVE_A_WIDTH, SCOPE_WAVE_A_HEIGHT));
    {
        unsigned crosses = 0u;
        for (i = 1u; i < SCOPE_WAVE_A_WIDTH; ++i) {
            CHECK(pixels[i] < SCOPE_WAVE_A_HEIGHT);
            if (pixels[i-1u] < SCOPE_WAVE_A_HEIGHT/2u && pixels[i] >= SCOPE_WAVE_A_HEIGHT/2u) ++crosses;
        }
        CHECK(crosses >= 3u && crosses <= 5u);
    }
}
static void check_other_shapes(void)
{
    uint32_t i;
    RmsResult r;
    float robust;
    Rms_Reset(&rms); Vpp_Reset(&vpp);
    for (i = 0u; i < 10000u; ++i) {
        uint16_t x = (i % 200u < 100u) ? 3248u : 848u;
        Rms_Push(&rms, x);
        Vpp_Push(&vpp, i == 333u ? 4095u : x);
        if (i < SCOPE_TRACE_SAMPLES) samples[i] = x;
    }
    r = Rms_Result(&rms); robust = Vpp_RobustCounts(&vpp);
    CHECK(fabsf(r.ac_counts - 1200.0f) < 1.0f);
    CHECK(fabsf(robust - 2400.0f) < 16.0f);
    SineFit_Start(&fit, samples, SCOPE_TRACE_SAMPLES, 200000.0f, 1000.0f);
    while (!SineFit_Step(&fit, 24u)) {}
    CHECK(!fit.valid);
    Rms_Reset(&rms); Vpp_Reset(&vpp);
    for (i = 0u; i < 10000u; ++i) {
        unsigned p = i % 200u;
        uint16_t x = (uint16_t)(848u + 24u * (p <= 100u ? p : 200u - p));
        Rms_Push(&rms, x); Vpp_Push(&vpp, x);
        if (i < SCOPE_TRACE_SAMPLES) samples[i] = x;
    }
    r = Rms_Result(&rms); robust = Vpp_RobustCounts(&vpp);
    CHECK(fabsf(r.ac_counts - 692.82f) < 2.0f);
    CHECK(fabsf(robust - 2400.0f) < 30.0f);
    SineFit_Start(&fit, samples, SCOPE_TRACE_SAMPLES, 200000.0f, 1000.0f);
    while (!SineFit_Step(&fit, 24u)) {}
    CHECK(!fit.valid);
    Frequency_Init(&frequency); Frequency_SetThreshold(&frequency, 2000u, 0u);
    Rms_Reset(&rms);
    for (i = 0u; i < 10000u; ++i) {
        Rms_Push(&rms, 2000u);
        CHECK(!Frequency_Push(&frequency, 2000u, i));
    }
    CHECK(Frequency_FinishWindow(&frequency, 200000u) == 0.0f);
    r = Rms_Result(&rms);
    CHECK(r.ac_counts == 0.0f && r.total_counts == 2000.0f);
}
static void check_frequency_steps(void)
{
    static const float hz[] = {100.0f,40000.0f,100.0f,12345.0f};
    uint32_t i, index = 0u;
    unsigned stage, window;
    Frequency_Init(&frequency);
    Frequency_SetThreshold(&frequency,2048u,2400u);
    for (stage = 0u; stage < 4u; ++stage) {
        float measured = 0.0f;
        for (window = 0u; window < 2u; ++window) {
            for (i = 0u; i < 10000u; ++i, ++index)
                (void)Frequency_Push(&frequency,sine_sample(hz[stage],index,0.314159f,2048.0f,0),index);
            measured = Frequency_FinishWindow(&frequency,200000u);
        }
        test_details[0] = hz[stage]; test_details[1] = measured;
        CHECK(fabsf(measured-hz[stage]) < hz[stage]*0.002f);
    }
}
static HmiProtocol hmi;
static HmiSnapshot snapshot;
static unsigned raw_packets, cle_packets, value_packets;
static int respond = 1, stop_in_first_data, validate_values = 1;
static void receive(const uint8_t *a, unsigned n)
{
    unsigned i;
    for (i = 0u; i < n; ++i) HmiProtocol_Receive(&hmi, a[i]);
}
static void ack(uint8_t code)
{
    uint8_t a[4] = {code,255u,255u,255u}; receive(a, 4u);
}
static int mock_send(void *context, const uint8_t *data, uint16_t n)
{
    (void)context;
    if (n >= 5u && memcmp(data, "addt ", 5u) == 0) {
        CHECK((memcmp(data,"addt 1,0,242",12u)==0) || (memcmp(data,"addt 15,0,239",13u)==0));
        if (respond) ack(0xfeu);
    } else if (n == SCOPE_WAVE_A_WIDTH || n == SCOPE_WAVE_B_WIDTH) {
        ++raw_packets;
        if (stop_in_first_data && raw_packets == 1u) {
            const uint8_t a[] = {0x65u,3u,17u,0u,255u,255u,255u}; receive(a,sizeof(a));
        }
        if (respond) ack(0xfdu);
    } else if (n >= 4u && memcmp(data,"cle ",4u)==0) ++cle_packets;
    else if (n >= 7u && memcmp(data,"t1.txt=",7u)==0) {
        char text[321];
        memcpy(text,data,n); text[n] = '\0';
        if (validate_values) {
        CHECK(strstr(text,"t1.txt=\"40.0kHz\"") != 0);
        CHECK(strstr(text,"t3.txt=\"0.684V\"") != 0);
        CHECK(strstr(text,"t5.txt=\"1.934V\"") != 0);
        CHECK(strstr(text,"t7.txt=\"100.0Hz\"") != 0);
        CHECK(strstr(text,"t9.txt=\"0.500V\"") != 0);
        CHECK(strstr(text,"t11.txt=\"1.414V\"") != 0);
        }
        ++value_packets;
    }
    HmiProtocol_TxComplete(&hmi);
    return 1;
}
static void check_hmi(void)
{
    const uint8_t page[] = {0x66u,3u,255u,255u,255u};
    const uint8_t automatic[] = {0x65u,3u,16u,0u,255u,255u,255u};
    const uint8_t press[] = {0x65u,3u,17u,1u,255u,255u,255u};
    uint32_t now;
    HmiProtocol_Init(&hmi,mock_send,0,0u);
    CHECK(!HmiProtocol_CanSubmit(&hmi));
    receive(page,sizeof(page)); receive(press,sizeof(press));
    CHECK(!hmi.paused);
    memset(&snapshot,0,sizeof(snapshot));
    snapshot.reading[0].frequency_valid = snapshot.reading[1].frequency_valid = 1u;
    snapshot.reading[0].frequency_tenths = 400000u;
    snapshot.reading[0].rms_mv = 684u; snapshot.reading[0].vpp_mv = 1934u;
    snapshot.reading[1].frequency_tenths = 1000u;
    snapshot.reading[1].rms_mv = 500u; snapshot.reading[1].vpp_mv = 1414u;
    CHECK(HmiProtocol_Submit(&hmi,&snapshot));
    stop_in_first_data = 1;
    for (now = 0u; now < 200u; ++now) HmiProtocol_Process(&hmi,now);
    CHECK(hmi.paused && raw_packets == 2u && cle_packets == 2u && value_packets == 1u);
    CHECK(!HmiProtocol_Submit(&hmi,&snapshot));
    for (; now < 2000u; ++now) HmiProtocol_Process(&hmi,now);
    CHECK(raw_packets == 2u && cle_packets == 2u);
    receive(automatic,sizeof(automatic));
    CHECK(!hmi.paused && (HmiProtocol_TakeEvents(&hmi) & HMI_EVENT_AUTO));
    CHECK(HmiProtocol_Submit(&hmi,&snapshot));
    respond = 0;
    for (; now < 5000u; ++now) HmiProtocol_Process(&hmi,now);
    CHECK(hmi.errors > 0u && hmi.page == 255u && !HmiProtocol_CanSubmit(&hmi));
}
TestCoreDebug test_core_debug;
TestDwt test_dwt;
static uint32_t mock_index, mock_tick, mock_overruns;
static int mock_ready, mock_gap;
uint32_t HAL_GetTick(void) { return mock_tick; }
void Error_Handler(void) { CHECK(0); }
int Acquisition_Start(void)
{
    mock_index = mock_tick = mock_overruns = 0u;
    mock_ready = mock_gap = 0;
    return 1;
}
int Acquisition_CopyLatest(uint32_t *copy, uint32_t *first_index, int *gap)
{
    unsigned i;
    if (!mock_ready) return 0;
    mock_ready = 0;
    *first_index = mock_index;
    *gap = mock_gap;
    mock_gap = 0;
    for (i = 0u; i < SCOPE_DMA_HALF_WORDS; ++i) {
        uint16_t a = sine_sample(100.0f,mock_index+i,0.314159f,2048.0f,0);
        uint16_t b = sine_sample(40000.0f,mock_index+i,0.314159f,2048.0f,0);
        copy[i] = (uint32_t)a | ((uint32_t)b << 16);
    }
    mock_index += SCOPE_DMA_HALF_WORDS;
    return 1;
}
int Acquisition_HasFault(void) { return 0; }
uint32_t Acquisition_Overruns(void) { return mock_overruns; }
int HmiUart_Init(void)
{
    HmiProtocol_Init(&hmi,mock_send,0,mock_tick);
    return 1;
}
void HmiUart_Process(void) { HmiProtocol_Process(&hmi,mock_tick); }
HmiProtocol *HmiUart_Protocol(void) { return &hmi; }
static void pipeline_blocks(unsigned count)
{
    unsigned b, poll;
    for (b = 0u; b < count; ++b) {
        mock_ready = 1;
        mock_tick = mock_index / 200u;
        for (poll = 0u; poll < 32u; ++poll) ScopeApp_Process();
    }
}
static void check_pipeline(void)
{
    const uint8_t page[] = {0x66u,3u,255u,255u,255u};
    const uint8_t stop[] = {0x65u,3u,17u,0u,255u,255u,255u};
    const uint8_t automatic[] = {0x65u,3u,16u,0u,255u,255u,255u};
    unsigned before, i;
    respond = 1; stop_in_first_data = validate_values = 0;
    CHECK(ScopeApp_Init());
    receive(page,sizeof(page));
    pipeline_blocks(160u);
    for (i = 0u; i < 2u; ++i) {
        HmiReading *r = &hmi.snapshot.reading[i];
        test_details[0] = (float)r->frequency_tenths;
        test_details[1] = (float)r->rms_mv;
        test_details[2] = (float)r->vpp_mv;
        CHECK(r->frequency_valid);
        CHECK(r->frequency_tenths >= (i == 0u ? 998u : 399900u));
        CHECK(r->frequency_tenths <= (i == 0u ? 1002u : 400100u));
        CHECK(r->rms_mv > 675u && r->rms_mv < 693u);
        CHECK(r->vpp_mv > 1910u && r->vpp_mv < 1960u);
    }
    CHECK(g_scope_diagnostics.displayed_frames >= 2u);
    CHECK(g_scope_diagnostics.dma_overruns == 0u);
    receive(stop,sizeof(stop));
    for (i = 0u; i < 32u; ++i) ScopeApp_Process();
    before = raw_packets;
    pipeline_blocks(50u);
    CHECK(raw_packets == before && hmi.paused);
    receive(automatic,sizeof(automatic));
    pipeline_blocks(80u);
    CHECK(raw_packets > before && !hmi.paused);
    mock_index += SCOPE_DMA_HALF_WORDS;
    mock_overruns = 1u; mock_gap = 1;
    pipeline_blocks(100u);
    CHECK(g_scope_diagnostics.dma_overruns == 1u);
    CHECK(hmi.snapshot.reading[0].frequency_valid && hmi.snapshot.reading[1].frequency_valid);
}
void test_start(void)
{
    static const float frequencies[] = {100.0f,123.45f,997.0f,1000.0f,12345.0f,39900.0f,40000.0f};
    unsigned i, phase;
    for (i = 0u; i < sizeof(frequencies)/sizeof(frequencies[0]); ++i) {
        for (phase = 0u; phase < 3u; ++phase) {
            ++test_case;
            check_sine(frequencies[i],0.314159f * phase,1600.0f + 200.0f * phase);
        }
    }
    ++test_case; check_other_shapes();
    ++test_case; check_frequency_steps();
    ++test_case; check_hmi();
    ++test_case; check_pipeline();
    test_passed = 1u;
    TEST_BREAK();
    for (;;) {}
}
#ifdef __CC_ARM
void SystemInit(void) {}
int main(void) { test_start(); return 0; }
#endif
