#include "system_app.h"
#define FS GPIO_PIN_12
#define CLK GPIO_PIN_13
#define DAT GPIO_PIN_15
static void word(uint16_t v){
    unsigned i;
    GPIOB->BRR=FS;
    for(i=0;i<16;i++,v<<=1){
        HAL_GPIO_WritePin(GPIOB,DAT,(v&0x8000)?GPIO_PIN_SET:GPIO_PIN_RESET);
        GPIOB->BRR=CLK; __NOP(); __NOP(); GPIOB->BSRR=CLK;
    }
    GPIOB->BSRR=FS;
}
void ad9833_init(void){word(0x2100);}
void ad9833_write(const Source *s){
    uint32_t ftw=(uint32_t)((((uint64_t)(s->hz+s->offset)<<28)+MCLK9833_HZ/2)/MCLK9833_HZ);
    word(0x4000|(ftw&0x3fff));word(0x4000|(ftw>>14));
    word(0xc000|((s->phase*4096+180)/360 % 4096));
    word(0x2000|(s->wave==1?2:s->wave==2?0x28:0));
}
