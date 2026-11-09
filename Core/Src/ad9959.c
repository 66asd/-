#include "system_app.h"
#define CS GPIO_PIN_6
#define CLK GPIO_PIN_7
#define DAT GPIO_PIN_8
#define UPDATE GPIO_PIN_9
#define RESET_PIN GPIO_PIN_5
static void reg(unsigned addr,uint32_t value,unsigned bits){
    unsigned n;
    uint64_t stream=((uint64_t)addr<<bits)|value;
    GPIOB->BRR=CS;
    for(n=bits+8;n;n--){
        HAL_GPIO_WritePin(GPIOB,DAT,(stream&((uint64_t)1<<(n-1)))?GPIO_PIN_SET:GPIO_PIN_RESET);
        GPIOB->BSRR=CLK; __NOP(); __NOP(); GPIOB->BRR=CLK;
    }
    GPIOB->BSRR=CS;
}
static void update(void){
    GPIOB->BSRR=UPDATE; for(volatile unsigned n=0;n<32;n++) __NOP(); GPIOB->BRR=UPDATE;
}
void ad9959_init(void){
    GPIOB->BSRR=RESET_PIN; HAL_Delay(1); GPIOB->BRR=RESET_PIN; HAL_Delay(1);
    reg(0,0x10,8);
    reg(1,(SYS9959_HZ>255000000UL?0x800000UL:0) | (PLL9959==1?0:(PLL9959<<18)),24);
    update();HAL_Delay(5);
    reg(3,0x000301,24); /* CH0 full DAC current, sine output */
}
void ad9959_write(const Source *s){
    reg(4,(uint32_t)((((uint64_t)(s->hz+s->offset)<<32)+SYS9959_HZ/2)/SYS9959_HZ),32);
    reg(5,(s->phase*16384+180)/360 % 16384,16);
    reg(6,0x1000|(s->percent*1023+50)/100,24);
    update();
}
