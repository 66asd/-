#include "system_app.h"
#include <math.h>
static DisplayFrame frames[2];
static unsigned back, ready;
static uint32_t sequence;
volatile uint32_t measurement_last_cycles, measurement_max_cycles;

void measurement_changed(void) { ready = 0; }

const DisplayFrame *measurement_acquire(void)
{
    if (!ready) return 0;
    const DisplayFrame *result = &frames[back];
    back ^= 1U;
    ready = 0;
    return result;
}
static unsigned channel(uint32_t pair,unsigned ch){
    return ch ? (pair>>16)&0xfff : pair&0xfff;
}
static uint32_t vpp_sample(const uint32_t *p, uint32_t count, uint32_t i,
                           unsigned ch, uint32_t window)
{
    uint32_t a, b, c, t; /* a/b/c是前/中/后原始码；t只用于交换，最后b返回三个码的中值。 */
    b = (p[i] >> (ch * 16U)) & 0xFFFU;
    /* 高频每周期点数少，不做中值；首尾缺少邻点时保持原值。 */
    if (window == 1U || i == 0U || i + 1U == count)
        return b;
    a = (p[i - 1U] >> (ch * 16U)) & 0xFFFU;
    c = (p[i + 1U] >> (ch * 16U)) & 0xFFFU;
    if (a > b)
    {
        t = a;
        a = b;
        b = t;
    }
    if (b > c)
        b = c;
    if (a > b)
        b = a;
    return b;
}

/* S1: align 128 phase bins using Q8 positions. */
static float filtered_span(const uint32_t *p, const Reading *r, unsigned raw_span)
{
    if (!r->period_q8) return raw_span;
    unsigned cycles = ((HALF_PAIRS-1U)*256U-r->first_q8)/r->period_q8;
    if (cycles > 32U) cycles = 32U;
    if (!cycles) return raw_span;
    unsigned median = r->period_q8 >= 128U*256U ? 3U : 1U;
    unsigned low = 4095U*256U*cycles, high = 0;
    unsigned bins = (r->period_q8 + 63U)/64U;
    if (bins > 128U) bins = 128U;
    for (unsigned bin = 0; bin < bins; ++bin) {
        unsigned pos = r->first_q8 + bin*r->period_q8/bins;
        unsigned sum = 0;
        for (unsigned c = 0; c < cycles; ++c, pos += r->period_q8) {
            unsigned i = pos >> 8, frac = pos & 255U;
            unsigned a = vpp_sample(p, HALF_PAIRS, i, 0, median);
            unsigned b = vpp_sample(p, HALF_PAIRS, i+1, 0, median);
            sum += a*(256U-frac) + b*frac;
        }
        if (sum < low) low = sum;
        if (sum > high) high = sum;
    }
    return (float)(high-low)/(256U*cycles);
}

static const int16_t quarter[65] = {0,25,50,75,100,125,150,175,200,224,249,273,297,321,345,369,392,415,438,460,483,505,526,548,569,590,610,630,650,669,688,706,724,742,759,775,792,807,822,837,851,865,878,891,903,915,926,936,946,955,964,972,980,987,993,999,1004,1009,1013,1016,1019,1021,1023,1024,1024};
static int sine_basis(unsigned phase)
{
    unsigned i = phase & 63U;
    int v = quarter[(phase & 64U) ? 64U-i : i];
    return (phase & 128U) ? -v : v;
}

static float sine_span(const uint32_t *pairs, const Reading *r, unsigned mid,
                       DisplayFrame *out)
{
    unsigned raw = out->raw_max - out->raw_min;
    out->residual_codes = 0;
    out->status = r->clipped ? FIT_CLIP : FIT_UNLOCKED;
    if (r->clipped || !r->period_q8) return raw;

    /* Eliminate the DC column from the sine/cosine normal equations. */
    int ss=0, cc=0, sc=0, sum_s=0, sum_c=0, sum_y=0;
    int64_t sy=0, cy=0, yy=0;
    uint32_t phase=0, increment=(uint32_t)((1ULL<<40)/r->period_q8);
    for (unsigned i=0; i<HALF_PAIRS; ++i, phase+=increment) {
        unsigned index=(phase+0x00800000U)>>24;
        int sn=sine_basis(index), cs=sine_basis(index+64U);
        int y=(int)channel(pairs[i],1)-(int)mid;
        sum_s+=sn; sum_c+=cs; sum_y+=y;
        ss+=sn*sn; cc+=cs*cs; sc+=sn*cs;
        sy+=sn*y; cy+=cs*y; yy+=y*y;
    }
    float n=HALF_PAIRS;
    float u=ss-(float)sum_s*sum_s/n, v=cc-(float)sum_c*sum_c/n;
    float w=sc-(float)sum_s*sum_c/n;
    float ys=sy-(float)sum_s*sum_y/n, yc=cy-(float)sum_c*sum_y/n;
    float det=u*v-w*w;
    if (det<=0) return raw;
    float a=(v*ys-w*yc)/det, b=(u*yc-w*ys)/det;
    float amplitude=1024.0f*sqrtf(a*a+b*b);
    float error=(yy-(float)sum_y*sum_y/n-a*ys-b*yc)/n;
    out->residual_codes=error>0 ? sqrtf(error) : 0;
    if (amplitude<6) return raw;
    if (out->residual_codes>2.0f+0.12f*amplitude) {
        out->status=FIT_DISTORTED;
        return raw;
    }
    out->status=FIT_OK;
    return 2*amplitude;
}

/* Display-only symmetric FIR. Keep sparse high-frequency samples intact. */
static unsigned plot_code(const uint32_t *pairs, unsigned i, unsigned ch,
                          unsigned radius)
{
    unsigned sum = channel(pairs[i], ch) * (radius + 1U);
    for (unsigned d = 1; d <= radius; ++d) {
        unsigned left = i >= d ? i - d : 0;
        unsigned right = i + d < HALF_PAIRS ? i + d : HALF_PAIRS - 1U;
        sum += (radius + 1U - d) * (channel(pairs[left], ch) + channel(pairs[right], ch));
    }
    unsigned weight = (radius + 1U) * (radius + 1U);
    return (sum + weight / 2U) / weight;
}

static void measure(const uint32_t *pairs, DisplayFrame *out){
    Reading *reading=out->reading;
    unsigned trigger=0,window=DEFAULT_WINDOW;
    for(unsigned ch=0;ch<2;ch++){
        unsigned lo=4095,hi=0,sum=0,nedge=0,first=0,last=0,prev=0,armed=0;
        
        Reading *r=reading+ch;
        for(unsigned i=0;i<HALF_PAIRS;i++){
            unsigned x=channel(pairs[i],ch);
            if(x<lo)lo=x;
            if(x>hi)hi=x;
            sum+=x;
        }
        r->clipped = lo <= 2 || hi >= 4093;
        if(ch){out->raw_min=lo; out->raw_max=hi;}
        unsigned mid=sum/HALF_PAIRS,band=(hi-lo)/10;
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
        unsigned begin=nedge>1?(first+255)/256:0;
        unsigned end=nedge>1?(last+255)/256:HALF_PAIRS;
        unsigned count=end-begin;
        uint64_t squares=0;
        sum=0;
        for(unsigned i=begin;i<end;++i){
            unsigned x=channel(pairs[i],ch);
            sum+=x; squares+=x*x;
        }
        float rms=sqrtf((float)(squares*count-(uint64_t)sum*sum))/count;
        float step=(float)vdda_mv*(ch ? GAIN2_PERMILLE : GAIN1_PERMILLE)/(4096.0f*1000);
        float span=ch ? sine_span(pairs,r,mid,out) : filtered_span(pairs,r,hi-lo);
        if (ch && out->status == FIT_UNLOCKED) {
            r->hz = 0;
            r->period_q8 = 0;
        }
        r->pp_mv=(unsigned)(span*step+0.5f);
        r->rms_mv=(unsigned)(rms*step+0.5f);
        if(ch==0){
            if(r->period_q8){
                window=(4*r->period_q8+128)/256;
                if(window>MAX_WINDOW) window=MAX_WINDOW;
                if(window<2) window=2;
            }
            if(nedge && first/256+window<=HALF_PAIRS) trigger=first/256;
        }
    }
    /* Shared time base: keep at least about 30 columns per fast cycle. */
    for (unsigned ch = 0; ch < 2; ++ch) {
        unsigned limit = (16U * reading[ch].period_q8 + 128U) / 256U;
        if (limit >= 2U && limit < window) window = limit;
    }
    /* One reusable filtered window for both channels; raw snapshot stays intact. */
    static uint16_t filtered[MAX_WINDOW];
    for (unsigned ch = 0; ch < 2; ++ch) {
        unsigned period = reading[ch].period_q8 / 256U;
        unsigned radius = period >= 100U ? 3U : period >= 50U ? 1U : 0U;
        unsigned low = 4095, high = 0;
        for (unsigned i = 0; i < window; ++i) {
            unsigned value = plot_code(pairs, trigger + i, ch, radius);
            filtered[i] = (uint16_t)value;
            if (value < low) low = value;
            if (value > high) high = value;
        }
        int span = (int)(high - low);
        if (span < 32) span = 32;
        for (unsigned col = 0; col < PLOT_WIDTH; ++col) {
            unsigned scale = PLOT_WIDTH - 1U;
            unsigned pos = col * (window - 1U);
            unsigned i = pos / scale;
            unsigned frac = pos % scale;
            unsigned a = filtered[i];
            unsigned b = frac ? filtered[i+1U] : a;
            unsigned value = (a * (scale - frac) + b * frac + scale / 2U) / scale;
            int centered = 2 * (int)value - (int)(low + high);
            out->trace[ch][col] = (uint8_t)(52 + centered * 44 / span);
        }
    }
}

void measurement_tick(void)
{
    const uint32_t *p = sample_take();
    if (!p) return;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    uint32_t start = DWT->CYCCNT;
    measure(p, &frames[back]);
    frames[back].sequence = ++sequence;
    ready = 1;
    measurement_last_cycles = DWT->CYCCNT - start;
    if (measurement_last_cycles > measurement_max_cycles)
        measurement_max_cycles = measurement_last_cycles;
}
