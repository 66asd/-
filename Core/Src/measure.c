#include "system_app.h"
#include <math.h>
#include <string.h>
#define PI2 6.28318530718f
#define PROFILE 128U
Reading reading[2];
uint8_t trace[2][PLOT_WIDTH];
static float profile[2][PROFILE];
static float periods[2], origins[2];
static float display_seconds;
static unsigned auto_time=1;
void measure_auto(void){auto_time=1;}
static unsigned code(const uint32_t *p, unsigned i, unsigned ch) {
    return (p[i] >> (ch * 16U)) & 4095U;
}
/* Four-point interpolation avoids the high-frequency loss of linear resampling. */
static float interp(const uint32_t *p, float pos, unsigned ch) {
    unsigned i = (unsigned)pos;
    float t = pos - i, b = (float)code(p,i,ch);
    float a = (float)code(p,i ? i-1U : i,ch);
    float c = (float)code(p,i+1U < HALF_PAIRS ? i+1U : i,ch);
    float d = (float)code(p,i+2U < HALF_PAIRS ? i+2U : HALF_PAIRS-1U,ch);
    return b + 0.5f*t*(c-a+t*(2*a-5*b+4*c-d+t*(3*(b-c)+d-a)));
}
void reset_span(void) { }
/* Both channels now use the same coherent snapshot, never stale extrema. */
int span_ready(const uint32_t *pairs) { (void)pairs; return 1; }

static void analyze(const uint32_t *p, unsigned ch) {
    Reading *r = &reading[ch];
    unsigned lo=4095, hi=0, n=0, armed=0;
    float first=0, last=0, st=0, skt=0, worst=0, prev_edge=0;
    float period=0, mean=0, rms=0, span;
    for (unsigned i=0;i<HALF_PAIRS;++i) {
        unsigned x=code(p,i,ch);
        if(x<lo)lo=x;
        if(x>hi)hi=x;
    }
    memset(r,0,sizeof(*r));
    r->clipped = lo<=8U || hi>=4087U;
    float mid=0.5f*(hi+lo), band=(hi-lo)*0.15f;
    if(band<4)band=4;
    for(unsigned i=1;i<HALF_PAIRS && hi-lo>=24U;++i) {
        float a=(float)code(p,i-1U,ch), b=(float)code(p,i,ch);
        if(b<mid-band)armed=1;
        if(armed && a<mid && b>=mid) {
            float t=i-1U+(mid-a)/(b-a);
            if(!n)first=t;
            else {
                float gap=t-prev_edge;
                if(n>1U) {
                    float error=fabsf(gap-(prev_edge-first)/(n-1U));
                    if(error>worst)worst=error;
                }
            }
            st+=t; skt+=n*t; last=t; prev_edge=t; ++n; armed=0;
        }
    }
    if(n>=2U) {
        /* Least-squares slope of all crossing times; retain fractional samples. */
        float k=0.5f*(n-1U);
        period=(skt-k*st)/(n*(n*(float)n-1.0f)/12.0f);
        /* Allow crossing quantization around the inclusive 100kHz/6-sample limit. */
        if(period<5.9f || sample_hz/period>SCOPE_MAX_HZ*1.01f || worst>period*0.15f)period=0;
    }
    if(period>0) {
        r->period_q8=(unsigned)(period*256.0f+0.5f);
        r->first_q8=(unsigned)(first*256.0f+0.5f);
        r->hz=(unsigned)(sample_hz/period+0.5f);
    }
    periods[ch]=period; origins[ch]=first;
    /* Integrate x and x^2 over complete cycles with fractional endpoints. */
    float begin=period>0?first:0, end=period>0?last:HALF_PAIRS-1U;
    float sum=0, squares=0;
    for(unsigned i=(unsigned)begin;i+1U<HALF_PAIRS && i<end;++i) {
        float l=begin>i?begin-i:0, h=end<i+1U?end-i:1;
        float a=(float)code(p,i,ch)-mid, b=(float)code(p,i+1U,ch)-mid;
        float w=h-l, q=0.5f*(h*h-l*l);
        sum+=a*w+(b-a)*q;
        squares+=a*a*w+(b*b-a*a)*q;
    }
    mean=sum/(end-begin);
    float variance=squares/(end-begin)-mean*mean;
    if(variance<0)variance=0;
    rms=sqrtf(variance); span=(float)(hi-lo);
    if(period>0) {
        unsigned cycles=(unsigned)((last-first)/period+0.5f);
        if(cycles>64U)cycles=64U;
        float low=4095, high=0;
        for(unsigned bin=0;bin<PROFILE;++bin) {
            float total=0; unsigned count=0;
            for(unsigned cy=0;cy<cycles;++cy) {
                float pos=first+period*(cy+bin/(float)PROFILE);
                if(pos>=HALF_PAIRS-2U)break;
                total+=interp(p,pos,ch); ++count;
            }
            float v=count?total/count:mid;
            /* Cubic interpolation must not invent overshoot on square edges. */
            if(v<lo)v=(float)lo;
            if(v>hi)v=(float)hi;
            profile[ch][bin]=v;
            if(v<low)low=v;
            if(v>high)high=v;
        }
        span=high-low;
        /* Fit DC+sin+cos only when residual confirms a clean sine. No DDS assumption.
           General waveforms retain the phase-averaged peak span and true AC RMS. */
        float ss=0, cc=0, sc=0, sy=0, cy=0, s=0, c=0, yy=0, ysum=0;
        unsigned count=0;
        float omega=PI2/period, sn=sinf(omega), cs=cosf(omega);
        float snv=sinf(omega*(ceilf(first)-first)), csv=cosf(omega*(ceilf(first)-first));
        for(unsigned i=(unsigned)ceilf(first);i<=last && i<HALF_PAIRS;++i) {
            float y=(float)code(p,i,ch)-mid;
            ss+=snv*snv; cc+=csv*csv; sc+=snv*csv;
            sy+=snv*y; cy+=csv*y; s+=snv; c+=csv; yy+=y*y; ysum+=y; ++count;
            float next=snv*cs+csv*sn;
            csv=csv*cs-snv*sn; snv=next;
        }
        float m=ysum/count;
        ss-=s*s/count; cc-=c*c/count; sc-=s*c/count;
        sy-=s*m; cy-=c*m; yy-=ysum*m;
        float det=ss*cc-sc*sc;
        if(det>1 && yy>1 && !r->clipped && period>=5.9f) {
            float a=(sy*cc-cy*sc)/det, b=(cy*ss-sy*sc)/det;
            float residual=yy-a*sy-b*cy;
            if(residual<yy*0.0004f) {
                float amp=sqrtf(a*a+b*b);
                span=2*amp; rms=amp*0.70710678118f;
                float dc=mid+m-(a*s+b*c)/count;
                for(unsigned bin=0;bin<PROFILE;++bin) {
                    float angle=PI2*bin/PROFILE;
                    profile[ch][bin]=dc+a*sinf(angle)+b*cosf(angle);
                }
            }
        }
    }
    float step=vdda_mv*(ch?GAIN2_PERMILLE:GAIN1_PERMILLE)/(4096.0f*1000.0f);
    r->pp_mv=(unsigned)(span*step+0.5f);
    r->rms_mv=(unsigned)(rms*step+0.5f);
}
void measure(const uint32_t *pairs) {
    analyze(pairs,0); analyze(pairs,1);
    unsigned ref=periods[0]>0?0:1;
    if(auto_time && periods[ref]>0){
        display_seconds=SCOPE_CYCLES*periods[ref]/sample_hz;
        auto_time=0;
    }
    float window=display_seconds>0?display_seconds*sample_hz:DEFAULT_WINDOW;
    if(window>HALF_PAIRS-2U)window=HALF_PAIRS-2U;
    if(window<2)window=2;
    float trigger=origins[ref];
    if(trigger+window>HALF_PAIRS-2U)trigger=HALF_PAIRS-2U-window;
    for(unsigned ch=0;ch<2;++ch) {
        float low=4095, high=0;
        if(periods[ch]>0) {
            for(unsigned i=0;i<PROFILE;++i) {
                if(profile[ch][i]<low)low=profile[ch][i];
                if(profile[ch][i]>high)high=profile[ch][i];
            }
        } else {
            for(unsigned i=0;i<HALF_PAIRS;++i) {
                float x=(float)code(pairs,i,ch);
                if(x<low)low=x;
                if(x>high)high=x;
            }
        }
        float span=high-low;
        if(span<32)span=32;
        unsigned width=ch?PLOT_WIDTH2:PLOT_WIDTH;
        for(unsigned col=0;col<width;++col) {
            float pos=trigger+col*(window-1)/(width-1U), v;
            if(periods[ch]>0) {
                float phase=(pos-origins[ch])/periods[ch];
                phase-=floorf(phase);
                float bin=phase*PROFILE;
                unsigned i=(unsigned)bin; float t=bin-i;
                v=profile[ch][i%PROFILE]*(1-t)+profile[ch][(i+1U)%PROFILE]*t;
            } else v=interp(pairs,pos,ch);
            int y=(int)(105+(v-0.5f*(low+high))*176/span+0.5f);
            if(y<17)y=17;
            if(y>193)y=193;
            trace[ch][col]=(uint8_t)y;
        }
        for(unsigned col=width;col<PLOT_WIDTH;++col)trace[ch][col]=105;
    }
}
