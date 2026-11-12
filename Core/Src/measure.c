#include "system_app.h"
#include <math.h>
#include <string.h>
#define PI2 6.28318530718f
#define PROFILE 128U
Reading reading[2];
uint8_t trace[2][PLOT_WIDTH];
/* Debugger diagnostics: 1 only when this frame passes the sine model checks. */
volatile unsigned sine_fit_used[2];
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

/* Exact 12-bit order statistics, in two 6-bit histogram passes. The complete
   multi-cycle record is used; discard 0.5% of samples at EACH end, not one peak.
   Only 256 bytes of temporary counters, no sorting buffer or dynamic allocation. */
static float quantile_span(const uint32_t *p, unsigned ch) {
    uint16_t hist[2][64]={{0}};
    unsigned rank[2]={HALF_PAIRS/200U,HALF_PAIRS-1U-HALF_PAIRS/200U};
    unsigned base[2]={0}, value[2]={0};
    for(unsigned i=0;i<HALF_PAIRS;++i)++hist[0][code(p,i,ch)>>6];
    for(unsigned end=0;end<2U;++end) {
        unsigned count=0;
        for(unsigned bin=0;bin<64U;++bin) {
            if(count+hist[0][bin]>rank[end]) {
                base[end]=bin; rank[end]-=count; break;
            }
            count+=hist[0][bin];
        }
    }
    memset(hist,0,sizeof(hist));
    for(unsigned i=0;i<HALF_PAIRS;++i) {
        unsigned x=code(p,i,ch);
        for(unsigned end=0;end<2U;++end)
            if((x>>6)==base[end])++hist[end][x&63U];
    }
    for(unsigned end=0;end<2U;++end) {
        unsigned count=0;
        for(unsigned bin=0;bin<64U;++bin) {
            count+=hist[end][bin];
            if(count>rank[end]) {value[end]=base[end]*64U+bin; break;}
        }
    }
    return (float)(value[1]-value[0]);
}

/* Small pivoted normal-equation solve. Fixed storage, no heap or sample buffer. */
static int solve(float m[4][5], unsigned n, float x[4]) {
    for(unsigned col=0;col<n;++col) {
        unsigned pivot=col;
        for(unsigned row=col+1U;row<n;++row)
            if(fabsf(m[row][col])>fabsf(m[pivot][col]))pivot=row;
        if(fabsf(m[pivot][col])<0.001f)return 0;
        for(unsigned k=col;k<=n;++k) {
            float t=m[col][k]; m[col][k]=m[pivot][k]; m[pivot][k]=t;
        }
        float div=m[col][col];
        for(unsigned k=col;k<=n;++k)m[col][k]/=div;
        for(unsigned row=0;row<n;++row)if(row!=col) {
            float factor=m[row][col];
            for(unsigned k=col;k<=n;++k)m[row][k]-=factor*m[col][k];
        }
    }
    for(unsigned i=0;i<n;++i)x[i]=m[i][n];
    return 1;
}
static void rotate(float *s, float *c, float sn, float cs, unsigned i) {
    float next=*s*cs+*c*sn;
    *c=*c*cs-*s*sn; *s=next;
    if((i&63U)==63U) {
        /* Correct oscillator roundoff without per-sample trigonometry. */
        float scale=1.5f-0.5f*((*s)*(*s)+(*c)*(*c));
        *s*=scale; *c*=scale;
    }
}
typedef struct { float a,b,dc,omega; } Sine;
static int fit_sine(const uint32_t *p, unsigned ch, float mid, float period, Sine *f) {
    const float center=(HALF_PAIRS-1U)*0.5f;
    float sigma=4096, initial=PI2/period;
    f->a=0; f->b=0; f->dc=0; f->omega=initial;
    /* First solve DC/sin/cos. Then jointly refine frequency and amplitude.
       All 1024 original samples participate; no phase-bin peak selection. */
    for(unsigned pass=0;pass<4U;++pass) {
        float m[4][5]={{0}}, dx[4]={0}, error=0;
        unsigned n=pass?4U:3U;
        float sn=sinf(f->omega), cs=cosf(f->omega);
        float s=sinf(-center*f->omega), c=cosf(-center*f->omega);
        float cutoff=fmaxf(2.0f,3.0f*sigma);
        for(unsigned i=0;i<HALF_PAIRS;++i) {
            float e=(float)code(p,i,ch)-mid-f->dc-f->a*s-f->b*c;
            float h[4]={s,c,1,((float)i-center)/HALF_PAIRS*(f->a*c-f->b*s)};
            /* Huber weights suppress isolated interference, not real amplitude steps. */
            float weight=fabsf(e)>cutoff?cutoff/fabsf(e):1;
            for(unsigned j=0;j<n;++j) {
                for(unsigned k=0;k<n;++k)m[j][k]+=weight*h[j]*h[k];
                m[j][n]+=weight*h[j]*e;
            }
            error+=weight*e*e;
            rotate(&s,&c,sn,cs,i);
        }
        if(!solve(m,n,dx))return 0;
        float scale=pass && fabsf(dx[3])>0.75f?0.75f/fabsf(dx[3]):1;
        f->a+=scale*dx[0]; f->b+=scale*dx[1]; f->dc+=scale*dx[2];
        if(pass)f->omega+=scale*dx[3]/HALF_PAIRS;
        sigma=sqrtf(error/HALF_PAIRS);
    }
    if(f->omega<=0 || fabsf(f->omega/initial-1)>0.02f)return 0;
    float amp=sqrtf(f->a*f->a+f->b*f->b), error=0, harmonic[4][2]={{0}};
    if(amp<12)return 0;
    float sn=sinf(f->omega), cs=cosf(f->omega);
    float s=sinf(-center*f->omega), c=cosf(-center*f->omega);
    float cutoff=fmaxf(2.0f,3.0f*sigma);
    for(unsigned i=0;i<HALF_PAIRS;++i) {
        float e=(float)code(p,i,ch)-mid-f->dc-f->a*s-f->b*c;
        error+=e*e;
        /* A noisy sine is acceptable; coherent 2nd..5th harmonics are not noise.
           Clip only this diagnostic's residual so a few spikes cannot mimic THD. */
        float residual=fmaxf(-cutoff,fminf(cutoff,e)), hs=s, hc=c;
        for(unsigned h=0;h<4U;++h) {
            float next=hs*c+hc*s; hc=hc*c-hs*s; hs=next;
            harmonic[h][0]+=residual*hs; harmonic[h][1]+=residual*hc;
        }
        rotate(&s,&c,sn,cs,i);
    }
    if(error>HALF_PAIRS*amp*amp*0.01f)return 0;
    /* Include the finite-record noise floor instead of rejecting noisy sine frames. */
    float limit=0.003f*amp+4*sigma*sqrtf(2.0f/HALF_PAIRS);
    for(unsigned h=0;h<4U;++h) {
        /* Harmonics at/above Nyquist cannot be distinguished from their aliases. */
        if((h+2U)*f->omega>=PI2*0.5f)continue;
        float magnitude=2*sqrtf(harmonic[h][0]*harmonic[h][0]+harmonic[h][1]*harmonic[h][1])/HALF_PAIRS;
        if(magnitude>limit)return 0;
    }
    f->dc+=mid;
    return 1;
}

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
    sine_fit_used[ch]=0;
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
    rms=sqrtf(variance); span=quantile_span(p,ch);
    if(period>0) {
        Sine fit;
        if(!r->clipped && fit_sine(p,ch,mid,period,&fit)) {
            float amp=sqrtf(fit.a*fit.a+fit.b*fit.b);
            span=2*amp; rms=amp*0.70710678118f;
            period=PI2/fit.omega;
            /* Locate a fitted rising crossing to keep waveform and timebase coherent. */
            first=(HALF_PAIRS-1U)*0.5f-atan2f(fit.b,fit.a)/fit.omega;
            first-=floorf(first/period)*period;
            periods[ch]=period; origins[ch]=first;
            r->hz=(unsigned)(sample_hz/period+0.5f);
            r->period_q8=(unsigned)(period*256+0.5f);
            r->first_q8=(unsigned)(first*256+0.5f);
            sine_fit_used[ch]=1;
            for(unsigned bin=0;bin<PROFILE;++bin)
                profile[ch][bin]=fit.dc+amp*sinf(PI2*bin/PROFILE);
        } else {
            /* Phase averaging is for drawing only; VPP remains the order statistic. */
            unsigned cycles=(unsigned)((last-first)/period+0.5f);
            if(cycles>64U)cycles=64U;
            for(unsigned bin=0;bin<PROFILE;++bin) {
                float total=0; unsigned count=0;
                for(unsigned cy=0;cy<cycles;++cy) {
                    float pos=first+period*(cy+bin/(float)PROFILE);
                    if(pos>=HALF_PAIRS-2U)break;
                    total+=interp(p,pos,ch); ++count;
                }
                /* Clamp drawing interpolation overshoot at sharp square edges. */
                profile[ch][bin]=fmaxf((float)lo,fminf((float)hi,count?total/count:mid));
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
