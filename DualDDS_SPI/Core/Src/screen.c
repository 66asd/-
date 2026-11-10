#include "system_app.h"
#include <stdio.h>
#include <string.h>
/* One complete ASCII command in flight. Only the main loop writes TX state. */
static uint8_t rx[256],tx[96];
static volatile uint8_t head,tail,overflow;
static volatile unsigned txpos,txlen;
static uint8_t frame[8],used;
static unsigned page=255,row,dirty=1,plot,paint;
static uint32_t nextframe,query;
void screen_uart_irq(void){
    uint32_t sr=USART1->SR;
    if(sr&(USART_SR_RXNE|USART_SR_ORE|USART_SR_NE|USART_SR_FE)){
        uint8_t value=(uint8_t)USART1->DR,next=(uint8_t)(head+1);
        if(next==tail || (sr&(USART_SR_ORE|USART_SR_NE|USART_SR_FE))) overflow=1;
        else {rx[head]=value;__DMB();head=next;}
    }
    if((sr&USART_SR_TXE) && (USART1->CR1&USART_CR1_TXEIE)){
        if(txpos<txlen) USART1->DR=tx[txpos++];
        else {USART1->CR1&=~USART_CR1_TXEIE; txlen=0;}
    }
}
static void send(const char *s){
    unsigned n=strlen(s);
    if(n+3>sizeof(tx))return;
    memcpy(tx,s,n);memset(tx+n,255,3);
    txpos=0;txlen=n+3;__DMB();USART1->CR1|=USART_CR1_TXEIE;
}
void screen_init(void){USART1->CR1|=USART_CR1_RXNEIE;}
void screen_invalidate(void){row=0;dirty=1;paint=0;nextframe=0;}
static void receive(uint8_t b){
    /* Recognize a valid suffix: a partial/foreign frame cannot swallow the next event. */
    if(used==sizeof(frame)){memmove(frame,frame+1,sizeof(frame)-1);--used;}
    frame[used++]=b;
    if(used<3 || memcmp(frame+used-3,"\xff\xff\xff",3))return;
    if(used>=7 && frame[used-7]==0x65 && frame[used-6]<=4 && frame[used-4]==0){
        unsigned p=frame[used-6]; page=p;settings_key(p,frame[used-5]);
    }else if(used>=5 && frame[used-5]==0x66 && frame[used-4]<=4){
        if(page!=frame[used-4]){page=frame[used-4];screen_invalidate();}
    }else if(used>=4 && frame[used-4]==0x88){page=255;screen_invalidate();query=0;}
    used=0;
}
static void textcmd(const char *name,const char *value){
    char cmd[80];snprintf(cmd,sizeof(cmd),"%s.txt=\"%s\"",name,value);send(cmd);
}
static void controls(void){
    static const char *const dnames[]={"t3","t4","t5","t6","t7","t8","b14","b15"};
    static const char *const shapes[]={"SIN","TRI","SQR"};
    static const char *const snames[]={"b1","b2","t11","t12","t13","t14","t15","t16"};
    char val[32];Source *s;
    if(page==2){
        switch(row){
        case 0:case 1:s=source+row;snprintf(val,sizeof(val),"%dHz",s->hz+s->offset);break;
        case 2:snprintf(val,sizeof(val),"%d%%",source[1].percent);break;
        case 3:case 5:snprintf(val,sizeof(val),"%dd",source[row==3].phase);break;
        case 4:strcpy(val,shapes[source[0].wave]);break;
        default:snprintf(val,sizeof(val),"Step %dHz",source[row-6].increment);break;
        }
        textcmd(dnames[row],val);
    }else if(page==4){
        if(row<2)strcpy(val,source[row].sweep?"STOP":"START");
        else{
            unsigned k=(row-2)%3;s=source+(row>=5);
            snprintf(val,sizeof(val),k==2?"%dms":"%dHz",k==0?s->span:k==1?s->step:s->dwell);
        }
        textcmd(snames[row],val);
    }else{dirty=0;return;}
    if(++row==8){row=0;dirty=0;}
}
static void meter(unsigned index){
    static const char *const names[]={"t1","t3","t5","t7","t9","t11","t13"};
    Reading *r=reading+(index>=3);char v[24];
    if(index==6){
        Reading *a=reading,*b=reading+1;
        unsigned diff=a->hz>b->hz?a->hz-b->hz:b->hz-a->hz;
        if(!a->hz || !b->hz || diff*200>a->hz)strcpy(v,"--");
        else{
            int lag=(int)a->first_q8-(int)b->first_q8;
            int angle=lag*360/(int)a->period_q8%360;
            if(angle>180)angle-=360;
            if(angle< -180)angle+=360;
            snprintf(v,sizeof(v),"%dd",angle);
        }
    }else if(index%3==0){
        if(r->hz)snprintf(v,sizeof(v),"%uHz",r->hz);else strcpy(v,"--");
    }else{
        unsigned m=index%3==1?r->rms_mv:r->pp_mv;
        if (r->clipped) strcpy(v,"CLIP");
        else snprintf(v,sizeof(v),"%umV",m);
    }
    textcmd(names[index],v);
}
void screen_tick(void){
    uint32_t now=HAL_GetTick();
    if(overflow){__disable_irq();tail=head;overflow=0;__enable_irq();used=0;query=0;page=255;screen_invalidate();}
    while(tail!=head){uint8_t b=rx[tail];++tail;receive(b);}
    if(txlen)return;
    if(!query || (int32_t)(now-query)>=0){send("sendme");query=now+500;return;}
    if(page==2||page==4){
        if(!dirty && page==2 && (source[0].sweep||source[1].sweep) && (int32_t)(now-nextframe)>=0){
            nextframe=now+200;dirty=1;
        }
        if(dirty)controls();
        return;
    }
    if(page!=3)return;
    if(!paint){
        
        const uint32_t *p=sample_take();if(!p || !span_ready(p))return;
        measure(p);paint=1;plot=0;
    }
    if(plot<7)meter(plot);
    else{
        unsigned i=(plot-7)/2,ch=(plot-7)%2;char cmd[32];
        snprintf(cmd,sizeof(cmd),"add %u,0,%u",ch?17:1,trace[ch][i]);send(cmd);
    }
    if(++plot==7+2*PLOT_WIDTH)paint=0;
}
