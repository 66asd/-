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
#include "sample_rate.h"
#include "vofa_raw.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "captured_sine.inc"

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
static VppCycleMeter cycle_vpp;
static SineFit fit;
static float generator_rate=200000.0f;
static uint32_t random_state = 1234567u;
static float noise(void)
{
    random_state = random_state * 1664525u + 1013904223u;
    return (float)((int32_t)(random_state >> 16) - 32768) / 32768.0f;
}
static uint16_t sine_sample(float hz, uint32_t index, float phase, float dc, int noisy)
{
    float x = dc + 1200.0f * sinf(6.28318530718f * hz * ((float)index / generator_rate) + phase);
    if (noisy) x += 3.0f * noise();
    if (x < 0.0f) x = 0.0f;
    if (x > 4095.0f) x = 4095.0f;
    return (uint16_t)(x + 0.5f);
}
static void check_sine(float hz, float phase, float dc)
{
    uint32_t i;
    uint16_t stride = Waveform_Stride(hz, (uint32_t)generator_rate, SCOPE_TRACE_SAMPLES);
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
    measured = Frequency_FinishWindow(&frequency, (uint32_t)generator_rate);
    result = Rms_Result(&rms);
    robust = Vpp_RobustCounts(&vpp);
    test_details[0] = hz; test_details[1] = measured;
    test_details[2] = result.ac_counts; test_details[3] = robust;
    CHECK(fabsf(measured - hz) < hz * 0.002f);
    CHECK(fabsf(result.ac_counts - 848.5281f) < 6.0f);
    CHECK(fabsf(result.mean_counts - dc) < 5.0f);
    CHECK(result.total_counts > result.ac_counts + 100.0f);
    CHECK(robust < 2430.0f && robust > 1500.0f);
    for (i = 0u; i < SCOPE_TRACE_SAMPLES; ++i)
        samples[i] = sine_sample(hz, i * stride, phase, dc, 1);
    samples[601] = 4095u;
    SineFit_Start(&fit, samples, SCOPE_TRACE_SAMPLES, generator_rate / stride, measured);
    for (i = 0u; i < 200u && !SineFit_Step(&fit, 24u); ++i) {}
    test_details[4] = fit.amplitude; test_details[5] = fit.residual_ratio;
    CHECK(fit.done && fit.valid);
    CHECK(fabsf(2.0f * fit.amplitude - 2400.0f) < 20.0f);
    CHECK(Waveform_Render(samples, SCOPE_TRACE_SAMPLES, generator_rate / stride,
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
static uint32_t mock_tick, mock_us, tx_due, total_points, clear_count, values_count;
static uint8_t pending[320];
static const uint8_t *pending_pointer;
static uint16_t pending_size;
static int tx_busy, return_rx=1;
static unsigned screen_a, screen_b;
static int overlays_restored;
static char screen_text[12][24];
static void receive(const uint8_t *p,unsigned n)
{
    unsigned i; for(i=0u;i<n;++i) HmiProtocol_Receive(&hmi,p[i]);
}
static int mock_send(void *context,const uint8_t *p,uint16_t n)
{
    (void)context;
    CHECK(!tx_busy && n>0u && n<=sizeof(pending));
    memcpy(pending,p,n); pending_pointer=p; pending_size=n; tx_busy=1;
    tx_due=mock_us+((uint32_t)n*10000000u+115199u)/115200u;
    return 1;
}
static void finish_tx(void)
{
    unsigned i=0u, start=0u;
    if(!tx_busy || (int32_t)(mock_us-tx_due)<0) return;
    CHECK(memcmp(pending,pending_pointer,pending_size)==0);
    while(i+2u<pending_size) {
        if(pending[i]==255u && pending[i+1u]==255u && pending[i+2u]==255u) {
            char cmd[128]; unsigned id,ch,value;
            CHECK(i-start<sizeof(cmd)); memcpy(cmd,pending+start,i-start); cmd[i-start]='\0';
            if(cmd[0]=='t') {
                unsigned field;
                const char *q=strchr(cmd,'"'), *end;
                if(q && sscanf(cmd,"t%u",&field)==1 && field<12u && (end=strchr(q+1,'"'))!=0) {
                    unsigned length=(unsigned)(end-q-1);
                    CHECK(length<sizeof(screen_text[0]));
                    memcpy(screen_text[field],q+1,length); screen_text[field][length]='\0';
                }
            }
            CHECK(strncmp(cmd,"addt",4u)!=0);
            if(sscanf(cmd,"add %u,%u,%u",&id,&ch,&value)==3) {
                CHECK(overlays_restored);
                CHECK((id==1u||id==15u)&&ch==0u);
                CHECK(value<(id==1u?SCOPE_WAVE_A_HEIGHT:SCOPE_WAVE_B_HEIGHT));
                if(id==1u) { if(screen_a<SCOPE_WAVE_A_WIDTH) ++screen_a; }
                else { if(screen_b<SCOPE_WAVE_B_WIDTH) ++screen_b; }
                ++total_points;
            } else if(sscanf(cmd,"cle %u,%u",&id,&ch)==2) {
                CHECK(ch==0u && (id==1u||id==15u));
                if(id==1u) screen_a=0u; else screen_b=0u;
                ++clear_count;
                overlays_restored=0;
            } else if(strcmp(cmd,"ref b1")==0) {
                overlays_restored=1;
            } else if(strncmp(cmd,"t1.txt=",7u)==0) ++values_count;
            else if((strcmp(cmd,"sendme")==0||strcmp(cmd,"page 3")==0) && return_rx) {
                const uint8_t page[]={0x66u,3u,255u,255u,255u}; receive(page,sizeof(page));
            }
            i+=3u; start=i;
        } else ++i;
    }
    CHECK(start==pending_size);
    tx_busy=0; HmiProtocol_TxComplete(&hmi);
}
static void ui_tick(uint32_t ms)
{
    mock_tick=ms; mock_us=ms*1000u; finish_tx(); HmiProtocol_Process(&hmi,ms);
}
static void check_hmi(void)
{
    uint32_t t, points;
    const uint8_t stop[]={0x65u,3u,17u,0u,255u,255u,255u};
    const uint8_t automatic[]={0x65u,3u,16u,0u,255u,255u,255u};
    const uint8_t wrong_press[]={0x65u,3u,17u,1u,255u,255u,255u};
    return_rx=0; tx_busy=0; mock_us=0u; total_points=clear_count=values_count=0u;
    HmiProtocol_Init(&hmi,mock_send,0,0u);
    CHECK(!HmiProtocol_CanSubmit(&hmi));
    for(t=0u;t<850u;++t) ui_tick(t);
    CHECK(HmiProtocol_CanSubmit(&hmi)); /* No RX or FE/FD response at all. */
    memset(&snapshot,0,sizeof(snapshot));
    memset(snapshot.a,100,sizeof(snapshot.a)); memset(snapshot.b,100,sizeof(snapshot.b));
    snapshot.reading[0].frequency_valid=1u; snapshot.reading[0].frequency_tenths=1200000u;
    snapshot.reading[0].rms_mv=742u; snapshot.reading[0].vpp_mv=2100u;
    CHECK(HmiProtocol_Submit(&hmi,&snapshot));
    for(;t<1900u;++t) ui_tick(t);
    CHECK(screen_a==242u && screen_b==239u && values_count==1u);
    CHECK(hmi.rx_bytes==0u && hmi.tx_bytes>6000u);
    receive(wrong_press,sizeof(wrong_press)); CHECK(!hmi.paused);
    CHECK(HmiProtocol_Submit(&hmi,&snapshot));
    for(;t<2100u;++t) ui_tick(t);
    receive(stop,sizeof(stop));
    for(;t<2150u;++t) ui_tick(t);
    points=total_points;
    for(;t<3000u;++t) ui_tick(t);
    CHECK(hmi.paused && total_points==points && !HmiProtocol_CanSubmit(&hmi));
    receive(automatic,sizeof(automatic)); CHECK(!hmi.paused);
    CHECK(HmiProtocol_TakeEvents(&hmi)&HMI_EVENT_AUTO);
    for(;t<3050u;++t) ui_tick(t);
    CHECK(HmiProtocol_Submit(&hmi,&snapshot));
    for(;t<4200u;++t) ui_tick(t);
    CHECK(total_points>points && screen_a==242u && screen_b==239u);
    CHECK(clear_count==2u); /* Continuous appends, not periodic clear-and-redraw. */
    tx_busy=0; HmiProtocol_TransportError(&hmi);
    for(;t<4300u;++t) ui_tick(t);
    CHECK(HmiProtocol_CanSubmit(&hmi));
}
static void check_rate_policy(void)
{
    static const float f[]={50,100,500,1000,2000,5000,10000,120000,180000};
#if SCOPE_TARGET_POINTS == 16
    static const uint32_t expected[]={5000u,5000u,10000u,20000u,50000u,100000u,200000u,500000u,500000u};
#else
    static const uint32_t expected[]={5000u,10000u,50000u,100000u,200000u,500000u,500000u,500000u,500000u};
#endif
    SampleRatePolicy p={0u,0u};
    unsigned i;
    for(i=0u;i<sizeof(f)/sizeof(f[0]);++i) CHECK(SampleRate_Target(f[i])==expected[i]);
    CHECK(SampleRate_Stride(500000u,50.0f)==100u);
    CHECK(SampleRate_Decide(&p,500000u,1000.0f,0.0f,0,0u)==500000u);
    CHECK(SampleRate_Decide(&p,500000u,1000.0f,0.0f,0,301u)==expected[3]);
    CHECK(SampleRate_Decide(&p,expected[3],120000.0f,50.0f,0,302u)==500000u);
    CHECK(SampleRate_Decide(&p,5000u,50.0f,0.0f,1,303u)==500000u);
}
TestCoreDebug test_core_debug;
TestDwt test_dwt;
static uint32_t mock_rate, mock_index, next_block_us, mock_overruns;
static float source_hz[2], source_phase[2],source_amplitude[2],source_noise[2],noise_phase[2];
static int mock_gap, start_failure, stalled;
static uint32_t lowest_rate;
static int mock_long_sampling;
uint32_t HAL_GetTick(void) { return mock_tick; }
void Error_Handler(void) { CHECK(0); }
int Acquisition_SetRate(uint32_t rate)
{
    CHECK(rate>=5000u && rate<=500000u);
    mock_rate=rate; mock_index=0u;
    next_block_us=mock_us+SCOPE_DMA_HALF_WORDS*1000000u/rate;
    if(rate<lowest_rate) lowest_rate=rate;
    return 1;
}
uint32_t Acquisition_Rate(void) { return mock_rate; }
uint16_t Acquisition_SampleCyclesX2(void)
{
    return !mock_long_sampling || mock_rate>200000u?15u:mock_rate>100000u?83u:mock_rate>20000u?143u:479u;
}
int Acquisition_SetLongSampling(int enabled)
{
    uint16_t previous=Acquisition_SampleCyclesX2();
    mock_long_sampling=enabled;
    return previous==Acquisition_SampleCyclesX2() || Acquisition_SetRate(mock_rate);
}
int Acquisition_Start(void)
{
    mock_long_sampling=0;
    mock_rate=0u; mock_overruns=0u; mock_gap=0;
    lowest_rate=500000u;
    if(start_failure) return 0;
    return Acquisition_SetRate(500000u);
}
int Acquisition_CopyLatest(uint32_t *copy,uint32_t *first_index,int *gap)
{
    unsigned i,ch;
    float step[2];
    if(stalled || mock_rate==0u || (int32_t)(mock_us-next_block_us)<0) return 0;
    *first_index=mock_index; *gap=mock_gap; mock_gap=0;
    for(ch=0u;ch<2u;++ch) {
        step[ch]=6.28318530718f*source_hz[ch]/mock_rate;
        step[ch]-=(unsigned)(step[ch]/6.28318530718f)*6.28318530718f;
    }
    for(i=0u;i<SCOPE_DMA_HALF_WORDS;++i) {
        uint32_t word=0u;
        for(ch=0u;ch<2u;++ch) {
            uint32_t x=(uint32_t)(2048.0f+source_amplitude[ch]*sinf(source_phase[ch])+
                source_noise[ch]*sinf(noise_phase[ch])+(source_noise[ch]>0.0f?3.0f*noise():0.0f)+0.5f);
            noise_phase[ch]+=6.28318530718f*137.0f/mock_rate;
            if(noise_phase[ch]>=6.28318530718f) noise_phase[ch]-=6.28318530718f;
            source_phase[ch]+=step[ch];
            if(source_phase[ch]>=6.28318530718f) source_phase[ch]-=6.28318530718f;
            word|=x<<(16u*ch);
        }
        copy[i]=word;
    }
    mock_index+=SCOPE_DMA_HALF_WORDS;
    next_block_us+=SCOPE_DMA_HALF_WORDS*1000000u/mock_rate;
    return 1;
}
int Acquisition_HasFault(void) { return 0; }
uint32_t Acquisition_Overruns(void) { return mock_overruns; }
int HmiUart_Init(void)
{
    VofaRaw_Init(0,0);
    memset(screen_text,0,sizeof(screen_text));
    tx_busy=0; HmiProtocol_Init(&hmi,mock_send,0,mock_tick); return 1;
}
void HmiUart_Process(void) { finish_tx(); HmiProtocol_Process(&hmi,mock_tick); }
HmiProtocol *HmiUart_Protocol(void) { return &hmi; }
static void run_ms(uint32_t duration)
{
    uint32_t end=mock_us+duration*1000u;
    while(mock_us<end) { mock_us+=50u; mock_tick=mock_us/1000u; ScopeApp_Process(); }
}
static void begin_pipeline(float a,float b)
{
    mock_us=mock_tick=0u; tx_busy=0; source_hz[0]=a; source_hz[1]=b;
    source_phase[0]=0.31f; source_phase[1]=0.62f;
    source_amplitude[0]=source_amplitude[1]=1200.0f*3300.0f/SCOPE_REFERENCE_MV;
    source_noise[0]=source_noise[1]=noise_phase[0]=noise_phase[1]=0.0f;
    return_rx=1; stalled=start_failure=0;
    CHECK(ScopeApp_Init());
}
static void check_readings(float a,float b)
{
    unsigned i;
    for(i=0u;i<2u;++i) {
        float f=i==0u?a:b;
        HmiReading *r=&hmi.snapshot.reading[i];
        test_details[0]=f; test_details[1]=r->frequency_tenths/10.0f;
        test_details[2]=(float)r->rms_mv; test_details[3]=(float)r->vpp_mv;
        CHECK(r->frequency_valid);
        CHECK(fabsf(r->frequency_tenths/10.0f-f)<f*0.012f);
        CHECK(r->rms_mv>660u && r->rms_mv<710u);
        CHECK(r->vpp_mv>1870u && r->vpp_mv<2000u);
    }
}
static void check_pipeline(void)
{
    uint32_t points;
    const uint8_t stop[]={0x65u,3u,17u,0u,255u,255u,255u};
    const uint8_t automatic[]={0x65u,3u,16u,0u,255u,255u,255u};
    begin_pipeline(50.0f,50.0f); run_ms(2200u);
    check_readings(50.0f,50.0f); CHECK(lowest_rate==5000u);
    source_hz[0]=120000.0f;
    run_ms(2400u); check_readings(120000.0f,50.0f);
    CHECK(Acquisition_Rate()==500000u);
    receive(stop,sizeof(stop)); run_ms(50u); points=total_points;
    run_ms(300u); CHECK(total_points==points && hmi.paused);
    receive(automatic,sizeof(automatic)); run_ms(1200u); CHECK(total_points>points);
    mock_gap=1; ++mock_overruns; run_ms(1200u);
    CHECK(g_scope_diagnostics.dma_overruns==1u);
    source_hz[0]=150000.0f; run_ms(1800u); check_readings(150000.0f,50.0f);
    source_hz[0]=50.0f; run_ms(2200u); check_readings(50.0f,50.0f);
    CHECK(lowest_rate==5000u);
    begin_pipeline(50.0f,100.0f); run_ms(2200u); check_readings(50.0f,100.0f);
    CHECK(lowest_rate<=10000u);
    begin_pipeline(50.0f,50.0f); stalled=1; return_rx=0;
    run_ms(1800u); CHECK(g_scope_diagnostics.displayed_frames>0u && total_points>points);
    mock_us=mock_tick=0u; start_failure=1; stalled=0; CHECK(ScopeApp_Init());
    run_ms(1800u); CHECK(g_scope_diagnostics.adc_fault && g_scope_diagnostics.displayed_frames>0u);
}
static void check_delayed_input(void)
{
    begin_pipeline(0.0f,0.0f); run_ms(1800u);
    CHECK(lowest_rate==5000u);
    CHECK(g_scope_diagnostics.analysis_rate_hz[0]>=5000u && g_scope_diagnostics.analysis_rate_hz[1]>=5000u);
    source_hz[0]=source_hz[1]=50.0f;
    run_ms(1800u); check_readings(50.0f,50.0f);
}

volatile float test_vpp_comparison[3];
static void check_one_volt(unsigned rate_index)
{
    static const float rates[]={20000.0f,100000.0f,500000.0f};
    unsigned i,phase;
    for(phase=0u;phase<6u;++phase) {
        uint8_t used;
        float raw,selected;
        Vpp_Reset(&vpp);
        for(i=0u;i<SCOPE_TRACE_SAMPLES;++i) {
            float angle=6.28318530718f*1000.0f*i/rates[rate_index]+0.17f*phase;
            float x=2048.0f+(0.5f*4095.0f/3.3f)*sinf(angle)+noise();
            samples[i]=(uint16_t)(x+0.5f);
        }
        samples[601]=4095u;
        for(i=0u;i<SCOPE_TRACE_SAMPLES;++i) Vpp_Push(&vpp,samples[i]);
        raw=Vpp_RobustCounts(&vpp);
        SineFit_Start(&fit,samples,SCOPE_TRACE_SAMPLES,rates[rate_index],1000.0f);
        while(!SineFit_Step(&fit,24u)) {}
        selected=Vpp_SelectCounts(raw,fit.amplitude*2.0f,fit.valid,1000.0f,rates[rate_index],&used);
        test_details[0]=rates[rate_index]; test_details[1]=selected*3300.0f/4095.0f;
        CHECK(used && fabsf(selected*3300.0f/4095.0f-1000.0f)<5.0f);
        test_vpp_comparison[rate_index]=selected*3300.0f/4095.0f;
    }
    {
        uint8_t used;
        CHECK(Vpp_SelectCounts(1000.0f,1251.0f,1,1000.0f,20000.0f,&used)==1000.0f && !used);
        CHECK(Vpp_SelectCounts(1700.0f,2400.0f,1,125000.0f,500000.0f,&used)==2400.0f && used);
    }
}
static void check_single_channel(void)
{
    float amplitude=500.0f*SCOPE_ADC_FULL_SCALE/SCOPE_REFERENCE_MV;
    begin_pipeline(1000.0f,1000.0f);
    source_amplitude[0]=0.0f; source_amplitude[1]=amplitude;
    run_ms(2400u);
    CHECK(!hmi.snapshot.reading[0].frequency_valid && hmi.snapshot.reading[1].frequency_valid);
    CHECK(hmi.snapshot.reading[1].frequency_tenths>9900u && hmi.snapshot.reading[1].frequency_tenths<10100u);
    CHECK(hmi.snapshot.reading[1].vpp_mv>=990u && hmi.snapshot.reading[1].vpp_mv<=1010u);
    CHECK(hmi.snapshot.reading[1].rms_mv>=347u && hmi.snapshot.reading[1].rms_mv<=361u);
    CHECK(g_scope_diagnostics.raw_min[0]==g_scope_diagnostics.raw_max[0]);
    CHECK(g_scope_diagnostics.raw_min[1]<g_scope_diagnostics.raw_max[1]);
    CHECK(lowest_rate==SampleRate_Target(1000.0f));
    CHECK(strcmp(screen_text[1],"--")==0 && strcmp(screen_text[5],"0.000V")==0);
    CHECK(screen_text[7][0]!=0 && strcmp(screen_text[7],"--")!=0);
    CHECK(screen_text[11][0]!=0 && strcmp(screen_text[11],"0.000V")!=0);
    source_amplitude[0]=amplitude; source_amplitude[1]=0.0f;
    run_ms(2200u);
    CHECK(hmi.snapshot.reading[0].frequency_valid && !hmi.snapshot.reading[1].frequency_valid);
    CHECK(hmi.snapshot.reading[0].vpp_mv>=990u && hmi.snapshot.reading[0].vpp_mv<=1010u);
    CHECK(strcmp(screen_text[11],"0.000V")==0 && strcmp(screen_text[5],"0.000V")!=0);
}

volatile float test_real_comparison[2][8];
static void check_distorted_sine(unsigned shape)
{
    unsigned i;
    for(i=0u;i<SCOPE_TRACE_SAMPLES;++i) {
        float a=6.28318530718f*1000.0f*i/20000.0f;
        float y=1200.0f*sinf(a);
        if(shape==0u) {
            if(y>960.0f) y=960.0f;
            if(y< -960.0f) y= -960.0f;
        } else y+=120.0f*sinf(3.0f*a);
        samples[i]=(uint16_t)(2048.0f+y+0.5f);
    }
    SineFit_Start(&fit,samples,SCOPE_TRACE_SAMPLES,20000.0f,1000.0f);
    while(!SineFit_Step(&fit,24u)) {}
    CHECK(!fit.valid);
}
static void check_real_capture(unsigned which)
{
    const uint16_t *x=which==0u?capture_3407:capture_3408;
    unsigned n=which==0u?sizeof(capture_3407)/sizeof(capture_3407[0]):sizeof(capture_3408)/sizeof(capture_3408[0]);
    unsigned i;
    uint8_t used;
    float hz,raw,selected;
    RmsResult r;
    Rms_Reset(&rms); Vpp_Reset(&vpp);
    for(i=0u;i<n;++i) { Rms_Push(&rms,x[i]); Vpp_Push(&vpp,x[i]); }
    r=Rms_Result(&rms); raw=Vpp_RobustCounts(&vpp);
    Frequency_Init(&frequency);
    Frequency_SetThreshold(&frequency,(uint16_t)(r.mean_counts+0.5f),(uint16_t)raw);
    for(i=0u;i<n;++i) (void)Frequency_Push(&frequency,x[i],i);
    hz=Frequency_FinishWindow(&frequency,20000.0f);
    SineFit_Start(&fit,x,(uint16_t)n,20000.0f,hz);
    while(!SineFit_Step(&fit,24u)) {}
    selected=Vpp_SelectCounts(raw,fit.amplitude*2.0f,fit.valid,hz,20000.0f,&used);
    test_details[0]=hz; test_details[1]=raw; test_details[2]=fit.amplitude*2.0f;
    test_details[3]=(float)fit.valid; test_details[4]=fit.residual_ratio;
    test_details[5]=selected; test_details[6]=(float)used;
    for(i=0u;i<7u;++i) test_real_comparison[which][i]=test_details[i];
    test_real_comparison[which][7]=selected*3190.0f/4095.0f;
    CHECK(hz>995.0f && hz<1005.0f);
    CHECK(fit.valid && used);
    CHECK(raw>1335.0f && raw<1350.0f);
    CHECK(selected>1250.0f && selected<1280.0f);
    CHECK(fit.residual_ratio>0.003f && fit.residual_ratio<0.005f);
}

volatile float test_small_comparison[4];
volatile float test_small_limits[4][2],test_small_pipeline_limits[2][2];
static void check_small_signal(unsigned mv, unsigned fs, unsigned shape)
{
    unsigned i,p;
    float amplitude=(float)mv*4095.0f/3190.0f/2.0f;
    for(p=0u;p<5u;++p) {
        float measured,span;
        Vpp_Reset(&vpp); Rms_Reset(&rms);
        CHECK(VppCycle_Start(&cycle_vpp,1024u,1000.0f,(float)fs));
        for(i=0u;i<1024u;++i) {
            float a=6.28318530718f*1000.0f*i/fs+0.137f*p;
            float y=sinf(a);
            if(shape==1u) y=y>=0.0f?1.0f:-1.0f;
            else if(shape==2u) y=2.0f/3.14159265359f*asinf(y);
            samples[i]=(uint16_t)(2118.0f+amplitude*y+
                20.0f*sinf(6.28318530718f*137.0f*i/fs+0.7f*p)+3.0f*noise()+0.5f);
            if(i==601u) samples[i]=4095u;
            if(i==701u) samples[i]=0u;
            Vpp_Push(&vpp,samples[i]); Rms_Push(&rms,samples[i]);
            VppCycle_Push(&cycle_vpp,samples[i]);
        }
        span=Vpp_RobustCounts(&vpp);
        measured=VppCycle_Result(&cycle_vpp)*3190.0f/4095.0f;
        test_details[0]=(float)mv; test_details[1]=(float)fs;
        test_details[2]=span*3190.0f/4095.0f; test_details[3]=measured;
        CHECK(fabsf(measured-mv)<(shape==2u?10.0f:4.0f));
        if(shape==0u) {
            unsigned result=(mv==50u?0u:2u)+(fs==100000u);
            test_small_comparison[result]=measured;
            if(p==0u || measured<test_small_limits[result][0]) test_small_limits[result][0]=measured;
            if(p==0u || measured>test_small_limits[result][1]) test_small_limits[result][1]=measured;
        }
    }
}
static void check_small_pipeline(void)
{
    unsigned i,t;
    begin_pipeline(1000.0f,1000.0f);
    source_amplitude[0]=25.0f*4095.0f/SCOPE_REFERENCE_MV;
    source_amplitude[1]=50.0f*4095.0f/SCOPE_REFERENCE_MV;
    source_noise[0]=source_noise[1]=20.0f;
    run_ms(2000u);
    for(t=0u;t<25u;++t) {
        run_ms(100u);
        for(i=0u;i<2u;++i) {
            unsigned target=i==0u?50u:100u;
            HmiReading *r=&hmi.snapshot.reading[i];
            test_details[0]=(float)target; test_details[1]=(float)r->vpp_mv;
            test_details[2]=r->frequency_tenths/10.0f;
            CHECK(r->frequency_valid && fabsf(r->frequency_tenths/10.0f-1000.0f)<15.0f);
            CHECK(fabsf((float)r->vpp_mv-target)<6.0f);
            if(t==0u || r->vpp_mv<test_small_pipeline_limits[i][0]) test_small_pipeline_limits[i][0]=(float)r->vpp_mv;
            if(t==0u || r->vpp_mv>test_small_pipeline_limits[i][1]) test_small_pipeline_limits[i][1]=(float)r->vpp_mv;
        }
    }
    CHECK(mock_long_sampling);
}

void test_start(void)
{
    static const float hz[]={50,75,100,150,200,300,500,750,1000,1500,2000,3000,8000,40000,80000,120000,125000,150000};
    unsigned i,phase;
    ++test_case;
    Vpp_Reset(&vpp);
    for(i=0u;i<1024u;++i) Vpp_Push(&vpp,(uint16_t)(2040u+i%64u));
    test_details[0]=Vpp_RobustCounts(&vpp);
    CHECK(test_details[0]==63.0f);
    for(i=0u;i<3u;++i) {
        ++test_case; check_small_signal(50u,20000u,i);
        ++test_case; check_small_signal(100u,20000u,i);
        ++test_case; check_small_signal(50u,100000u,i);
        ++test_case; check_small_signal(100u,100000u,i);
    }
    ++test_case; check_small_pipeline();
    for(i=0u;i<2u;++i) { ++test_case; check_real_capture(i); }
    for(i=0u;i<2u;++i) { ++test_case; check_distorted_sine(i); }
    for(i=0u;i<sizeof(hz)/sizeof(hz[0]);++i) {
        generator_rate=(float)SampleRate_Target(hz[i]);
        for(phase=0u;phase<3u;++phase) { ++test_case; check_sine(hz[i],0.314159f*phase,1600.0f+200.0f*phase); }
    }
    generator_rate=200000.0f;
    ++test_case; check_other_shapes();
    ++test_case; check_frequency_steps();
    ++test_case; check_rate_policy();
    ++test_case; check_hmi();
    ++test_case; check_pipeline();
    ++test_case; check_delayed_input();
    for(i=0u;i<3u;++i) { ++test_case; check_one_volt(i); }
    ++test_case; check_single_channel();
    test_passed=1u; TEST_BREAK(); for(;;) {}
}
#ifdef __CC_ARM
void SystemInit(void) {}
int main(void) { test_start(); return 0; }
#endif
