#include "system_app.h"
#include <math.h>
#include <string.h>
#include "sine_table.h"
#define PI2 6.28318530718f
#define PROFILE 128U
Reading reading[2];
uint8_t trace[2][PLOT_WIDTH];
/* Debugger diagnostics: 1 only when this frame passes the sine model checks. */
volatile unsigned sine_fit_used[2];
volatile unsigned measurement_cycles[2], measurement_ms;
volatile unsigned measurement_method[2], measurement_vpp_q8[2], measurement_vdda_mv;
volatile unsigned vpp_window_frames[2],vpp_window_cycles[2],vpp_ready[2],vpp_uncertainty_uv[2];
static float frame_vpp_mv[2];
static struct {
    float value[VPP_WINDOWS_MAX],mean,sd;
    uint16_t cycles[VPP_WINDOWS_MAX];
    unsigned count,next,hz;
} vpp_stats[2];
static float profile[2][PROFILE];
static float periods[2], origins[2];
static float display_seconds;
static unsigned auto_time=1;
void measure_auto(void){
    auto_time=1;memset(vpp_stats,0,sizeof(vpp_stats));
    memset((void *)vpp_ready,0,sizeof(vpp_ready));
}
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
static int sine_q10(uint32_t phase) {
    unsigned index=phase>>24, fraction=(phase>>8)&65535U;
    int a=sine_table[index], delta=sine_table[index+1U]-a;
    int value=a+(delta*(int)fraction)/65536;
    return (value+(value>=0?16:-16))/32;
}
typedef struct { float a,b,dc,omega; } Sine;
static int fit_sine(const uint32_t *p, unsigned ch, float mid, float period, Sine *f) {
    const int center=(int)(mid+0.5f);
    float sigma=4096, initial=PI2/period;
    f->a=0; f->b=0; f->dc=0; f->omega=initial;
    /* Keep the four-parameter fit, but use integer inner products on the F103
       (no FPU). Only the tiny equation solve uses float. Every pass uses 1024 points. */
    for(unsigned pass=0;pass<3U;++pass) {
        float m[4][5]={{0}}, dx[4]={0};
        int32_t normal[4][4]={{0}};
        int64_t rhs[4]={0}; uint64_t error=0;
        unsigned n=pass?4U:3U;
        float amp=sqrtf(f->a*f->a+f->b*f->b);
        if(amp>4096 || fabsf(f->dc)>4096)return 0;
        if(amp<1)amp=1;
        int aq=(int)(f->a*256), bq=(int)(f->b*256), dq=(int)(f->dc*256);
        int an=(int)(f->a/amp*32768), bn=(int)(f->b/amp*32768);
        int cutoff=(int)(fmaxf(2.0f,3.0f*sigma)*256);
        uint32_t step=(uint32_t)(f->omega*(4294967296.0f/PI2));
        uint32_t phase=0U-(uint32_t)((uint64_t)step*(HALF_PAIRS-1U)/2U);
        for(unsigned i=0;i<HALF_PAIRS;++i) {
            int s=sine_q10(phase), c=sine_q10(phase+0x40000000U);
            int e=((int)code(p,i,ch)-center)*256-(aq*s+bq*c)/1024-dq;
            int absolute=e<0?-e:e;
            int weight=absolute>cutoff?cutoff*256/absolute:256;
            int h[4]={s,c,1024,((an*c-bn*s)/32768)*(2*(int)i-(int)(HALF_PAIRS-1U))/(2*(int)HALF_PAIRS)};
            for(unsigned j=0;j<n;++j) {
                int weighted=weight*h[j];
                for(unsigned k=j;k<n;++k)normal[j][k]+=(weighted*h[k])/256;
                rhs[j]+=(int64_t)weighted*e;
            }
            int whole=e/256;
            error+=(uint64_t)(whole*whole)*weight;
            phase+=step;
        }
        for(unsigned j=0;j<n;++j) {
            for(unsigned k=j;k<n;++k)m[j][k]=m[k][j]=normal[j][k]/1048576.0f;
            m[j][n]=(float)rhs[j]/67108864.0f;
        }
        if(!solve(m,n,dx))return 0;
        float delta=dx[3]/amp;
        float scale=pass && fabsf(delta)>0.75f?0.75f/fabsf(delta):1;
        f->a+=scale*dx[0]; f->b+=scale*dx[1]; f->dc+=scale*dx[2];
        if(pass)f->omega+=scale*delta/HALF_PAIRS;
        sigma=sqrtf((float)error/(HALF_PAIRS*256.0f));
    }
    if(f->omega<=0 || fabsf(f->omega/initial-1)>0.02f)return 0;
    float amp=sqrtf(f->a*f->a+f->b*f->b);
    if(amp<12 || amp>4096 || fabsf(f->dc)>4096)return 0;
    uint64_t error=0; int64_t harmonic[4][2]={{0}};
    int aq=(int)(f->a*256), bq=(int)(f->b*256), dq=(int)(f->dc*256);
    int cutoff=(int)(fmaxf(2.0f,3.0f*sigma)*256);
    uint32_t step=(uint32_t)(f->omega*(4294967296.0f/PI2));
    uint32_t phase=0U-(uint32_t)((uint64_t)step*(HALF_PAIRS-1U)/2U);
    for(unsigned i=0;i<HALF_PAIRS;++i) {
        int s=sine_q10(phase), c=sine_q10(phase+0x40000000U);
        int e=((int)code(p,i,ch)-center)*256-(aq*s+bq*c)/1024-dq;
        error+=(uint64_t)((int64_t)e*e);
        int residual=e>cutoff?cutoff:e<-cutoff?-cutoff:e;
        for(unsigned h=0;h<4U;++h) {
            uint32_t hp=phase*(h+2U);
            harmonic[h][0]+=(int64_t)residual*sine_q10(hp);
            harmonic[h][1]+=(int64_t)residual*sine_q10(hp+0x40000000U);
        }
        phase+=step;
    }
    if((float)error/(HALF_PAIRS*65536.0f)>amp*amp*0.01f)return 0;
    /* Include the finite-record noise floor instead of rejecting noisy sine frames. */
    float limit=0.003f*amp+4*sigma*sqrtf(2.0f/HALF_PAIRS);
    for(unsigned h=0;h<4U;++h) {
        /* Harmonics at/above Nyquist cannot be distinguished from their aliases. */
        if((h+2U)*f->omega>=PI2*0.5f)continue;
        float a=(float)harmonic[h][0]/(HALF_PAIRS*262144.0f);
        float b=(float)harmonic[h][1]/(HALF_PAIRS*262144.0f);
        float magnitude=2*sqrtf(a*a+b*b);
        if(magnitude>limit)return 0;
    }
    f->dc+=center;
    return 1;
}

static float ac_rms(const uint32_t *p, unsigned ch, float mid, float begin, float end) {
    float sum=0, squares=0;
    for(unsigned i=(unsigned)begin;i+1U<HALF_PAIRS && i<end;++i) {
        float l=begin>i?begin-i:0, h=end<i+1U?end-i:1;
        float a=(float)code(p,i,ch)-mid, b=(float)code(p,i+1U,ch)-mid;
        float w=h-l, q=0.5f*(h*h-l*l);
        sum+=a*w+(b-a)*q;
        squares+=a*a*w+(b*b-a*a)*q;
    }
    float mean=sum/(end-begin);
    return sqrtf(fmaxf(0,squares/(end-begin)-mean*mean));
}
static void phase_profile(const uint32_t *p, unsigned ch, float first, float period,
                          unsigned cycles, unsigned lo, unsigned hi) {
    float values[64];
    if(cycles>64U)cycles=64U;
    for(unsigned bin=0;bin<PROFILE;++bin) {
        unsigned count=0;
        for(unsigned cy=0;cy<cycles;++cy) {
            float pos=first+period*(cy+bin/(float)PROFILE);
            if(pos>=HALF_PAIRS-2U)break;
            float v=interp(p,pos,ch); unsigned at=count;
            while(at && values[at-1U]>v){values[at]=values[at-1U];--at;}
            values[at]=v;++count;
        }
        /* Trim within each phase across cycles, not across the waveform's peaks. */
        unsigned trim=count/8U; float total=0;
        for(unsigned i=trim;i<count-trim;++i)total+=values[i];
        float v=count?total/(count-2U*trim):0.5f*(hi+lo);
        profile[ch][bin]=fmaxf((float)lo,fminf((float)hi,v));
    }
}
static int harmonic_profile(unsigned ch, float period) {
    float a[7],b[7],model[PROFILE],mean=0,error=0;
    unsigned harmonics=(unsigned)((period-0.25f)*0.5f);
    if(harmonics>7U)harmonics=7U;
    if(harmonics<4U)return 0;
    for(unsigned i=0;i<PROFILE;++i)mean+=profile[ch][i]/PROFILE;
    /* The folded phase grid is uniform: Fourier bases are orthogonal. Fit the
       coherent harmonics here, rather than treating them as random ADC noise. */
    for(unsigned h=0;h<harmonics;++h) {
        int64_t sy=0,cy=0; int32_t ss=0,cc=0;
        for(unsigned i=0;i<PROFILE;++i) {
            uint32_t phase=(i<<25)*(h+1U);
            int s=sine_q10(phase),c=sine_q10(phase+0x40000000U);
            int y=(int)((profile[ch][i]-mean)*256);
            sy+=(int64_t)y*s;cy+=(int64_t)y*c;ss+=s*s;cc+=c*c;
        }
        a[h]=(float)sy*4/ss;b[h]=(float)cy*4/cc;
    }
    for(unsigned i=0;i<PROFILE;++i) {
        float value=mean;
        for(unsigned h=0;h<harmonics;++h) {
            uint32_t phase=(i<<25)*(h+1U);
            value+=(a[h]*sine_q10(phase)+b[h]*sine_q10(phase+0x40000000U))/1024;
        }
        model[i]=value;
        float residual=profile[ch][i]-value;error+=residual*residual/PROFILE;
    }
    float fundamental=sqrtf(a[0]*a[0]+b[0]*b[0]);
    float independent=fminf((float)PROFILE,period);
    float limit=0.003f*fundamental+8*sqrtf(2*error/independent);
    /* Significant energy at the truncation boundary signals sharp edges (e.g.
       square/triangle); retain their folded profile instead of a Gibbs model. */
    for(unsigned h=harmonics-2U;h<harmonics;++h)
        if(sqrtf(a[h]*a[h]+b[h]*b[h])>limit)return 0;
    if(error>fundamental*fundamental*0.005f)return 0;
    memcpy(profile[ch],model,sizeof(model));
    return 1;
}
static float profile_quantile_span(unsigned ch) {
    float ordered[PROFILE];
    for(unsigned i=0;i<PROFILE;++i) {
        float value=profile[ch][i];unsigned at=i;
        while(at && ordered[at-1U]>value){ordered[at]=ordered[at-1U];--at;}
        ordered[at]=value;
    }
    /* Hazen plotting positions (i+1/2)/N: interpolate 0.5% and 99.5%, including
       the finite phase-grid spacing. No raw or folded maximum-minus-minimum. */
    float rank=0.005f*PROFILE-0.5f;
    unsigned index=(unsigned)rank;float fraction=rank-index;
    float low=ordered[index]+fraction*(ordered[index+1U]-ordered[index]);
    float high=ordered[PROFILE-1U-index]+fraction*(ordered[PROFILE-2U-index]-ordered[PROFILE-1U-index]);
    return high-low;
}
static void analyze(const uint32_t *p, unsigned ch) {
    Reading *r = &reading[ch];
    unsigned lo=4095, hi=0, n=0, armed=0;
    float first=0, last=0, st=0, skt=0, worst=0, prev_edge=0;
    float period=0, rms=0, span=0;
    for (unsigned i=0;i<HALF_PAIRS;++i) {
        unsigned x=code(p,i,ch);
        if(x<lo)lo=x;
        if(x>hi)hi=x;
    }
    memset(r,0,sizeof(*r));
    sine_fit_used[ch]=0;
    measurement_cycles[ch]=0;
    measurement_method[ch]=0;
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
    if(period>0) {
        Sine fit;
        measurement_cycles[ch]=(unsigned)((HALF_PAIRS-1U)/period);
        if(!r->clipped && measurement_cycles[ch]>=SCOPE_MIN_CYCLES && fit_sine(p,ch,mid,period,&fit)) {
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
            measurement_method[ch]=1;
            for(unsigned bin=0;bin<PROFILE;++bin)
                profile[ch][bin]=fit.dc+amp*sine_q10(bin<<25)/1024.0f;
        } else {
            unsigned cycles=(unsigned)((last-first)/period+0.5f);
            phase_profile(p,ch,first,period,cycles,lo,hi);
            measurement_method[ch]=3;
            if(!r->clipped && measurement_cycles[ch]>=SCOPE_MIN_CYCLES && harmonic_profile(ch,period))
                measurement_method[ch]=2;
            span=profile_quantile_span(ch);
        }
    }
    if(!sine_fit_used[ch]) {
        if(!period)span=quantile_span(p,ch);
        rms=ac_rms(p,ch,mid,period>0?first:0,period>0?last:HALF_PAIRS-1U);
    }
    measurement_vpp_q8[ch]=(unsigned)(span*256+0.5f);
    float step=measurement_vdda_mv*(ch?GAIN2_PERMILLE:GAIN1_PERMILLE)/(4096.0f*1000.0f);
    frame_vpp_mv[ch]=span*step;
    r->pp_mv=(unsigned)(span*step+0.5f);
    r->rms_mv=(unsigned)(rms*step+0.5f);
}
void measure(const uint32_t *pairs) {
    uint32_t start=HAL_GetTick();
    measurement_vdda_mv=vdda_mv;
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
    measurement_ms=HAL_GetTick()-start;
}
void measure_publish(void) {
    /* Called only after rate selection: short/obsolete probe frames never enter
       this statistical window. Each entry is an independent multi-cycle estimate. */
    for(unsigned ch=0;ch<2U;++ch) {
        unsigned count=vpp_stats[ch].count;
        float value=frame_vpp_mv[ch];
        int valid=reading[ch].hz && !reading[ch].clipped && measurement_cycles[ch]>=SCOPE_MIN_CYCLES;
        if(!valid) {
            vpp_stats[ch].count=0;vpp_stats[ch].next=0;
            vpp_window_frames[ch]=0;vpp_window_cycles[ch]=0;vpp_ready[ch]=1;
            vpp_uncertainty_uv[ch]=0;
            continue;
        }
        float threshold=fmaxf(4*vpp_stats[ch].sd,VPP_ERROR_FRACTION*vpp_stats[ch].mean);
        unsigned hz=reading[ch].hz;
        if(count && ((hz>vpp_stats[ch].hz?hz-vpp_stats[ch].hz:vpp_stats[ch].hz-hz)>hz/50U ||
           (count>=VPP_WINDOWS_MIN && fabsf(value-vpp_stats[ch].mean)>threshold))) {
            count=0;vpp_stats[ch].next=0;
        }
        unsigned at=vpp_stats[ch].next;
        vpp_stats[ch].value[at]=value;vpp_stats[ch].cycles[at]=measurement_cycles[ch];
        vpp_stats[ch].next=(at+1U)%VPP_WINDOWS_MAX;
        if(count<VPP_WINDOWS_MAX)++count;
        vpp_stats[ch].count=count;vpp_stats[ch].hz=hz;
        float mean=0,variance=0;unsigned cycles=0;
        for(unsigned i=0;i<count;++i){mean+=vpp_stats[ch].value[i]/count;cycles+=vpp_stats[ch].cycles[i];}
        for(unsigned i=0;i<count;++i){float d=vpp_stats[ch].value[i]-mean;variance+=d*d;}
        float sd=count>1U?sqrtf(variance/(count-1U)):0;
        vpp_stats[ch].mean=mean;vpp_stats[ch].sd=sd;
        /* Algorithm-only 3-sigma standard error plus integer-mV rounding. This
           excludes the meter/reference/ADC gain uncertainty; it is not calibration. */
        float uncertainty=3*sd/sqrtf((float)count)+0.5f;
        vpp_window_frames[ch]=count;vpp_window_cycles[ch]=cycles;
        vpp_uncertainty_uv[ch]=(unsigned)(uncertainty*1000+0.5f);
        vpp_ready[ch]=count>=VPP_WINDOWS_MIN && uncertainty<=VPP_ERROR_FRACTION*mean;
        reading[ch].pp_mv=(unsigned)(mean+0.5f);
    }
}
