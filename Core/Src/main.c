#include "system_app.h"
int main(void){
    HAL_Init(); board_init(); settings_init(); sample_start(); screen_init();
    for(;;){settings_tick(); screen_tick();}
}
