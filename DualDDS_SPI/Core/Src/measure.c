#include "system_app.h"
#include <math.h>
Reading reading[2];
uint8_t trace[2][PLOT_WIDTH];
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

/* 通用Vpp：按测得周期将多周期对齐，同相位求平均，再找峰谷；返回值仍是ADC码差。 */
static float filtered_span(const uint32_t *p, uint32_t count, unsigned ch,
                           float fs, float hz, float first, float last, unsigned raw_span)
{
    /* bin是128个相位位置的序号；cycle是周期序号；cycles是可叠加周期数。
     * i为左侧样本下标；median=3启用三点中值，=1直接取原码。 */
    uint32_t bin, cycle, cycles, i, median;
    /* period为每周期点数；pos为待插值的位置；a/b为左右两点；sum为同相位累加。
     * v为该相位的周期平均码；low/high记录平均后波形的峰谷，不是原始极端噪声点。 */
    float period, pos, sum, a, b, v, low = 4095, high = 0;

    /* 没有可靠周期时不做对齐，保留幅度观察，频率错误仍由原来的标志报告。 */
    if (hz <= 0)
        return raw_span;
    period = fs / hz; /* 500Hz对应400点/周期，10kHz对应20点/周期（当fs=200k）。 */
    cycles = (uint32_t)((last - first) / period + 0.5f);
    if (cycles == 0U)
        return raw_span;
    if (cycles > 32U) cycles = 32U; /* 稳态波最多平均32个周期，控制高频计算时间。 */
    median = period >= 128.0f ? 3U : 1U; /* 点数足够才做中值，避免高频峰顶被邻点明显压低。 */

    /* 一个周期分成128个相位；先平均各周期的同相位点，再找峰谷。
     * 不把整帧中碰巧最大的噪声点当成峰值，也不需要额外的采样数组。
     */
    for (bin = 0; bin < 128U; ++bin)
    {
        pos = first + period * bin / 128.0f;
        sum = 0;
        for (cycle = 0; cycle < cycles; ++cycle)
        {
            i = (uint32_t)pos;
            if (i + 1U >= count)
                break;
            a = (float)vpp_sample(p, count, i, ch, median);
            b = (float)vpp_sample(p, count, i + 1U, ch, median);
            /* 相位通常落在两个样本之间，线性插值避免直接取整造成抖动。 */
            sum += a + (b - a) * (pos - i);
            pos += period;
        }
        if (cycle == 0U)
            continue;
        v = sum / cycle;
        if (v < low)
            low = v;
        if (v > high)
            high = v;
    }
    /* shortcut: 适合稳定周期波；突发信号需要另设测量方式。 */
    return high - low;
}

/* Four consecutive DMA halves reproduce the 4096-point CH0 extrema window
   without a second raw buffer. At short sweep dwells publish one settled half. */
static unsigned blocks, low2, high2;
void reset_span(void) { blocks = 0; }
int span_ready(const uint32_t *pairs)
{
    if (!blocks) { low2 = 4095; high2 = 0; }
    for (unsigned i = 0; i < HALF_PAIRS; ++i) {
        unsigned x = channel(pairs[i], 1);
        if (x < low2) low2 = x;
        if (x > high2) high2 = x;
    }
    ++blocks;
    if (blocks < 4 && !source[0].sweep && !source[1].sweep) return 0;
    blocks = 0;
    return 1;
}
void measure(const uint32_t *pairs){
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
        r->clipped = ch && (low2 <= 2 || high2 >= 4093);
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
        float step=ch ? VDDA_MV*GAIN2_PERMILLE/(4095.0f*1000)
                      : vdda_mv*GAIN1_PERMILLE/(4096.0f*1000);
        float span=ch ? high2-low2 : filtered_span(pairs,HALF_PAIRS,0,FS_HZ,
            r->period_q8 ? FS_HZ*256.0f/r->period_q8 : 0,first/256.0f,last/256.0f,hi-lo);
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
    /* Independent vertical zoom; reserve 8 pixels above and below each trace.
       Flat or noise-only inputs retain a minimum range instead of filling the screen. */
    for (unsigned ch = 0; ch < 2; ++ch) {
        unsigned low = 4095, high = 0;
        for (unsigned i = 0; i < window; ++i) {
            unsigned code = channel(pairs[trigger+i], ch);
            if (code < low) low = code;
            if (code > high) high = code;
        }
        int span = (int)(high-low);
        if (span < 32) span = 32;
        for (unsigned col = 0; col < PLOT_WIDTH; ++col) {
            unsigned pos = trigger + col*(window-1)/(PLOT_WIDTH-1);
            int centered = 2*(int)channel(pairs[pos],ch) - (int)(low+high);
            trace[ch][col] = (uint8_t)(52 + centered*44/span);
        }
    }
}
