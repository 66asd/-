#include "vofa_raw.h"
#include "hmi_uart.h"
#include "usart.h"
#include <string.h>

volatile uint32_t test_failed_line, test_case, test_passed;
#define CHECK(x) do { if(!(x)) { test_failed_line=__LINE__; __breakpoint(0); for(;;) {} } } while(0)
UART_HandleTypeDef huart1={USART1};
static uint32_t tick, aborts;
static uint8_t *rx, *inflight, saved[320], busy, refuse;
static uint16_t size;
static uint32_t packed[512];

uint32_t HAL_GetTick(void) { return tick; }
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{
    CHECK(u==&huart1 && n<=sizeof(saved));
    if(busy || refuse) { refuse=0u; return HAL_BUSY; }
    inflight=p; size=n; memcpy(saved,p,n); busy=1u; return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{
    CHECK(u==&huart1 && n==1u); rx=p; return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *u)
{
    CHECK(u==&huart1); busy=0u; ++aborts; return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Abort(UART_HandleTypeDef *u)
{
    rx=0; return HAL_UART_AbortTransmit(u);
}
static void finish(void)
{
    CHECK(busy && memcmp(inflight,saved,size)==0);
    busy=0u; HAL_UART_TxCpltCallback(&huart1);
}
static void command(const char *s)
{
    while(*s) { CHECK(rx); *rx=(uint8_t)*s++; HAL_UART_RxCpltCallback(&huart1); }
    HmiUart_Process();
}
static void check_segment(unsigned segment, unsigned fs)
{
    unsigned i,j,index=0u;
    while(index<512u) {
        HmiUart_Process(); CHECK(busy && size==160u);
        for(i=0u;i<8u;++i) {
            float values[4];
            memcpy(values,inflight+i*20u,16u);
            CHECK(values[0]==(float)(((segment-1u)*512u+index)&4095u));
            CHECK(values[1]==(float)fs && values[2]==(float)segment && values[3]==(float)index);
            CHECK(memcmp(inflight+i*20u+16u,"\x00\x00\x80\x7f",4u)==0);
            ++index;
        }
        /* Neither capture nor busy processing may overwrite the in-flight bytes. */
        for(j=0u;j<512u;++j) packed[j]=0xFFFF0000u;
        VofaRaw_Offer(packed,512u,500000u);
        HmiUart_Process(); finish(); tick+=14u;
    }
    HmiUart_Process();
    CHECK(!busy && g_vofa_diagnostics.segments_sent==segment);
    CHECK(g_vofa_diagnostics.samples_sent==segment*512u);
}
int main(void)
{
    unsigned i,s;
    ++test_case;
    CHECK(HmiUart_Init() && !VofaRaw_Enabled());
    tick=800u; HmiUart_Process(); CHECK(busy);
    command("noiseVO"); CHECK(!VofaRaw_Enabled());
    command("FA"); CHECK(VofaRaw_Enabled() && aborts==1u && size==4u);
    CHECK(memcmp(inflight,"\x00\x00\x80\x7f",4u)==0); finish();
    /* Across eight captures every 12-bit ADC1 code is transmitted exactly. */
    for(s=1u;s<=8u;++s) {
        ++test_case;
        for(i=0u;i<512u;++i) packed[i]=((4095u-i)<<16)|((s-1u)*512u+i);
        VofaRaw_Offer(packed,511u,20000u); HmiUart_Process(); CHECK(!busy);
        VofaRaw_Offer(packed,512u,0u); HmiUart_Process(); CHECK(!busy);
        VofaRaw_Offer(packed,512u,20000u);
        memset(packed,0,sizeof(packed));
        refuse=1u; HmiUart_Process(); CHECK(!busy);
        check_segment(s,20000u);
    }
    ++test_case;
    VofaRaw_Offer(packed,512u,100000u); HmiUart_Process(); CHECK(busy);
    tick+=201u; HmiUart_Process();
    CHECK(g_vofa_diagnostics.errors==1u && busy && size==4u);
    finish();
    ++test_case;
    VofaRaw_Offer(packed,512u,500000u); HmiUart_Process(); CHECK(size==160u);
    command("HMI!"); CHECK(!VofaRaw_Enabled() && busy && size>3u);
    CHECK(inflight[0]==255u && inflight[1]==255u && inflight[2]==255u);
    CHECK(memcmp(inflight+3u,"page 3",6u)==0); finish();
    CHECK(HmiProtocol_CanSubmit(HmiUart_Protocol()));
    CHECK(g_vofa_diagnostics.segments_sent==8u);
    test_passed=1u; __breakpoint(0); for(;;) {}
}
void SystemInit(void) {}
