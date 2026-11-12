#include "system_app.h"
Source source[2];
static int clamp(int x,int lo,int hi){return x<lo?lo:x>hi?hi:x;}
static void write_source(unsigned ch)
{
#if DDS_ENABLED
    const Source *s = &source[ch];
    uint32_t hz = (uint32_t)(s->hz + s->offset);
    if (ch == 0)
        check(AD9833_SetOutput(hz, s->phase, (AD9833_Wave)s->wave));
    else
        check(AD9959_SetCH0(hz, s->phase, s->percent));
#else
    (void)ch;
#endif
}

static void apply(unsigned ch){
    write_source(ch);
    sample_changed(); screen_invalidate();
}
void settings_init(void){
    if (DDS_ENABLED) { check(AD9833_Init()); check(AD9959_Init()); }
    for(unsigned i=0;i<2;i++){
        Source *s=source+i;
        s->hz=i?DEFAULT_F2:DEFAULT_F1; s->wave=i?0:1; s->percent=100;
        s->increment=F_STEP; s->span=SWEEP_SPAN; s->step=SWEEP_STEP; s->dwell=SWEEP_MS;
        apply(i);
    }
}
static void frequency(Source *s,int sign){
    s->sweep=0; s->offset=0; s->hz=clamp(s->hz+sign*s->increment,F_MIN,F_MAX);
    s->span=clamp(s->span,0,F_MAX-s->hz);
}
static void phase(Source *s,int sign){s->phase=(s->phase+360+sign*PHASE_STEP)%360;}
static void dds_key(unsigned id){
    unsigned ch=(id>=10 && id<=15)||id==25;
    Source *s=source+ch;
    switch(id){
    case 8:case 10: frequency(s,1);break;
    case 9:case 11: frequency(s,-1);break;
    case 12:case 13:s->percent=clamp(s->percent+(id==12?AMP_STEP:-AMP_STEP),0,100);break;
    case 14:case 21:phase(s,1);break;
    case 15:case 22:phase(s,-1);break;
    case 18:case 19:case 20:s->wave=id-18;break;
    case 24:case 25:
        s->increment=s->increment>=1000?1:s->increment*10;
        screen_invalidate();return;
    default:return;
    }
    apply(ch);
}
static void sweep_key(unsigned id){
    unsigned ch=id==8||id>=21;
    Source *s=source+ch;
    if(id==7||id==8){s->sweep=!s->sweep;s->offset=0;}
    else if(id>=15 && id<=26){
        unsigned k=(id-15)%6; int sign=(k%2)?-1:1;
        switch(k/2){
        case 0:s->span=clamp(s->span+sign*SPAN_DELTA,0,F_MAX-s->hz);
               s->offset=clamp(s->offset,0,s->span);break;
        case 1:s->step=clamp(s->step+sign*STEP_DELTA,1,F_MAX-F_MIN);break;
        default:s->dwell=clamp(s->dwell-sign*SPEED_DELTA_MS,DWELL_MIN,DWELL_MAX);break;
        }
    }else return;
    s->next=HAL_GetTick()+s->dwell;apply(ch);
}
void settings_key(unsigned page,unsigned id){
    if(page==2) dds_key(id);
    else if(page==4) sweep_key(id);
    else if(page==3) scope_key(id);
}
void settings_tick(void){
    uint32_t now=HAL_GetTick();
    for(unsigned i=0;i<2;i++){
        Source *s=source+i;
        if(s->sweep && (int32_t)(now-s->next)>=0){
            s->offset=s->offset>=s->span?0:clamp(s->offset+s->step,0,s->span);
            s->next=now+s->dwell;
            write_source(i);
            sample_changed();
        }
    }
}
