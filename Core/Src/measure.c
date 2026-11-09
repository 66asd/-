#include "system_app.h"
#include <math.h>
Reading reading[2];
uint8_t trace[2][PLOT_WIDTH];
static unsigned channel(uint32_t pair,unsigned ch){
    return ch ? (pair>>16)&0xfff : pair&0xfff;
}
static unsigned mv(unsigned counts,unsigned ch){
    unsigned gain=ch?GAIN2_PERMILLE:GAIN1_PERMILLE;
    return (unsigned)(((uint64_t)counts*VDDA_MV*gain+2047500)/4095000);
}
void measure(const uint32_t *pairs){
    unsigned trigger=0,window=DEFAULT_WINDOW;
    for(unsigned ch=0;ch<2;ch++){
        unsigned lo=4095,hi=0,sum=0,nedge=0,first=0,last=0,prev=0,armed=0;
        uint64_t squares=0;
        Reading *r=reading+ch;
        for(unsigned i=0;i<HALF_PAIRS;i++){
            unsigned x=channel(pairs[i],ch);
            if(x<lo)lo=x;
            if(x>hi)hi=x;
            sum+=x;squares+=x*x;
        }
        unsigned mid=(hi+lo)/2,band=(hi-lo)/10;
        for(unsigned i=0;i<HALF_PAIRS;i++){
            unsigned x=channel(pairs[i],ch);
            if(x+band<mid) armed=1;
            if(armed && i && prev<mid && x>=mid && hi-lo>=16){
                last=(i-1)*256+(mid-prev)*256/(x-prev);
                if(!nedge) first=last;
                ++nedge;armed=0;
            }
            prev=x;
        }
        r->period_q8=nedge>1?(last-first)/(nedge-1):0;
        r->hz=r->period_q8?(FS_HZ*256+r->period_q8/2)/r->period_q8:0;
        r->first_q8=first;
        r->pp_mv=mv(hi-lo,ch);
        /* AC RMS. Subtraction in integer space preserves small signals on bias. */
        r->rms_mv=mv((unsigned)(sqrtf((float)(squares*HALF_PAIRS-(uint64_t)sum*sum))/HALF_PAIRS+0.5f),ch);
        if(ch==0){
            if(r->period_q8){
                window=(4*r->period_q8+128)/256;
                if(window>MAX_WINDOW) window=MAX_WINDOW;
                if(window<2) window=2;
            }
            if(nedge && first/256+window<=HALF_PAIRS) trigger=first/256;
        }
    }
    for(unsigned col=0;col<PLOT_WIDTH;col++){
        unsigned pos=trigger+col*(window-1)/(PLOT_WIDTH-1);
        for(unsigned ch=0;ch<2;ch++) trace[ch][col]=(channel(pairs[pos],ch)*104+2047)/4095;
    }
}
