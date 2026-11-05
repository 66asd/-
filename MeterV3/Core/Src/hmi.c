#include "hmi.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

static uint8_t rx_byte, rx_command;
static volatile uint8_t parser;
static volatile uint8_t rx_fault;
static volatile HmiRequest pending_request;
volatile uint32_t g_hmi_rx_errors;

typedef enum { VIEW_UNKNOWN, VIEW_NORMAL, VIEW_DIAGNOSTIC } HmiView;
static HmiView current_view = VIEW_UNKNOWN;

static HAL_StatusTypeDef command(const char *s)
{
    uint8_t data[96];
    size_t n = strlen(s);
    if (n > sizeof(data) - 3U)
        return HAL_ERROR;
    memcpy(data, s, n);
    /* 屏幕靠3个FF判断命令结束；不发送C字符串尾部的0。 */
    data[n++] = 0xFF;
    data[n++] = 0xFF;
    data[n++] = 0xFF;
    return HAL_UART_Transmit(&huart1, data, (uint16_t)n, 50U);
}

static HAL_StatusTypeDef text(const char *name, const char *value)
{
    char b[80];
    int n = snprintf(b, sizeof(b), "%s.txt=\"%s\"", name, value);
    if (n < 0 || n >= (int)sizeof(b))
        return HAL_ERROR;
    return command(b);
}

static void fixed2(char *b, size_t size, float value, float scale, const char *unit)
{
    long v;
    unsigned long m;
    const char *sign = "";

    /* 先换单位、放大100倍并四舍五入，再拆整数/小数。
     * 例如0.08905V乘1000得到89.05mV，最后输出89.05mV。
     * 用整数格式化，不要求Keil启用printf的浮点输出支持。
     */
    if (value < 0)
        v = (long)(value * scale * 100.0f - 0.5f);
    else
        v = (long)(value * scale * 100.0f + 0.5f);

    if (v < 0)
    {
        sign = "-";
        v = -v;
    }
    m = (unsigned long)v;
    snprintf(b, size, "%s%lu.%02lu%s", sign, m / 100UL, m % 100UL, unit);
}

HAL_StatusTypeDef Hmi_Init(void)
{
    parser = 0;
    rx_fault = 0;
    pending_request = HMI_REQUEST_NONE;
    current_view = VIEW_UNKNOWN;
    if (HAL_UART_Receive_IT(&huart1, &rx_byte, 1) != HAL_OK)
    {
        rx_fault = 1;
        return HAL_ERROR;
    }
    if (command("page 0") != HAL_OK)
        return HAL_ERROR;
    current_view = VIEW_NORMAL;
    return HAL_OK;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *h)
{
    if (h->Instance != USART1)
        return;

    /* 屏幕只发3字节按钮命令：A5 01 5A测量，A5 02 5A切换诊断。
     * 中断里只识别这3个字节、保存请求，不计算、不向屏幕发送。
     */
    if (parser == 0) /* 等帧头。 */
    {
        if (rx_byte == 0xA5)
            parser = 1;
    }
    else if (parser == 1) /* 等命令编号。 */
    {
        if (rx_byte == 1 || rx_byte == 2)
        {
            rx_command = rx_byte;
            parser = 2;
        }
        else if (rx_byte != 0xA5)
            parser = 0;
    }
    else /* 等帧尾。错误字节若是A5，就把它当作下一帧的开头。 */
    {
        if (rx_byte == 0x5A)
            pending_request = (HmiRequest)rx_command;
        parser = 0;
        if (rx_byte == 0xA5)
            parser = 1;
    }
    if (HAL_UART_Receive_IT(h, &rx_byte, 1) != HAL_OK)
        rx_fault = 1;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *h)
{
    if (h->Instance == USART1)
    {
        ++g_hmi_rx_errors;
        rx_fault = 1;
    }
}

HmiRequest Hmi_TakeRequest(void)
{
    HmiRequest request;
    uint32_t mask;
    if (rx_fault)
    {
        HAL_UART_AbortReceive(&huart1);
        parser = 0;
        pending_request = HMI_REQUEST_NONE;
        rx_fault = 0;
        if (HAL_UART_Receive_IT(&huart1, &rx_byte, 1) != HAL_OK)
            rx_fault = 1;
    }

    /* 取走并清空请求必须一起完成，避免清空时把刚到的新请求也抹掉。
     * 这里只关中断几条指令，并恢复进入前的状态，不覆盖上层的关中断设置。
     */
    mask = __get_PRIMASK();
    __disable_irq();
    request = pending_request;
    pending_request = HMI_REQUEST_NONE;
    __set_PRIMASK(mask);
    return request;
}

static HAL_StatusTypeDef select_view(uint8_t diagnostic)
{
    const char *names[] = {"t0", "t1", "t2", "t3"};
    const char *labels[] = {"S1 DC/mV", "S1 ALL", "S1 Vpp", "S2 DC/mV"};
    unsigned i;
    if (diagnostic)
    {
        if (current_view == VIEW_DIAGNOSTIC)
            return HAL_OK;
        current_view = VIEW_UNKNOWN;
        for (i = 0; i < 4; ++i)
        {
            if (text(names[i], labels[i]) != HAL_OK)
                return HAL_ERROR;
        }
        current_view = VIEW_DIAGNOSTIC;
    }
    else if (current_view != VIEW_NORMAL)
    {
        /* 退出诊断时重载一次，恢复你在HMI里设置的标题。
         * 正常刷新不重载页面，避免闪屏和覆盖你修改的标题。
         */
        if (command("page 0") != HAL_OK)
            return HAL_ERROR;
        current_view = VIEW_NORMAL;
    }
    return HAL_OK;
}

static const char *reason(uint32_t flags)
{
    if (flags & MEAS_FLAG_CAPTURE)
        return "ADC ERR";
    if (flags & MEAS_FLAG_CLIP)
        return "CLIP";
    if (flags & MEAS_FLAG_LOW)
        return "LOW SIG";
    return "NO FREQ";
}

static HAL_StatusTypeDef show_value(const char *name, float value,
                                    float scale, const char *unit, uint32_t flags)
{
    char b[24];
    if (flags != 0U)
        return text(name, reason(flags));
    fixed2(b, sizeof(b), value, scale, unit);
    return text(name, b);
}

static uint32_t voltage_flags(const MeterResult *r, unsigned ch)
{
    /* 没测出频率不一定妨碍观察幅度；只挡住采集失败、幅度过低和削顶。 */
    return (r->flags & MEAS_FLAG_CAPTURE) |
           (r->channel_flags[ch] & (MEAS_FLAG_LOW | MEAS_FLAG_CLIP));
}

static HAL_StatusTypeDef show_normal(const MeterResult *r)
{
    uint32_t capture_error = r->flags & MEAS_FLAG_CAPTURE;
    uint32_t frequency_error = r->channel_flags[0];
    /* 频率栏有FREQ时优先提示NO FREQ，让它与幅值栏的CLIP明确区分。 */
    if (frequency_error & MEAS_FLAG_FREQ)
        frequency_error = MEAS_FLAG_FREQ;

    if (show_value("t4", r->ac_rms_v[0], 1000, "mV", voltage_flags(r, 0)) != HAL_OK)
        return HAL_ERROR;
    if (show_value("t5", r->ac_rms_v[1], 1000, "mV", voltage_flags(r, 1)) != HAL_OK)
        return HAL_ERROR;
    if (show_value("t6", r->frequency_hz[0], 1, "Hz", capture_error | frequency_error) != HAL_OK)
        return HAL_ERROR;
    return show_value("t7", r->phase_deg, 1, "deg", r->flags);
}

static HAL_StatusTypeDef show_diagnostic(const MeterResult *r)
{
    uint32_t flags = r->flags & MEAS_FLAG_CAPTURE;
    /* 诊断时保留原始计算量，帮助区分直流、交流RMS和总RMS。 */
    if (show_value("t4", r->dc_v[0], 1000, "mV", flags) != HAL_OK)
        return HAL_ERROR;
    if (show_value("t5", r->total_rms_v[0], 1000, "mV", flags) != HAL_OK)
        return HAL_ERROR;
    if (show_value("t6", r->vpp_v[0], 1000, "mV", flags) != HAL_OK)
        return HAL_ERROR;
    return show_value("t7", r->dc_v[1], 1000, "mV", flags);
}

HAL_StatusTypeDef Hmi_Show(const MeterResult *r, uint8_t diagnostic)
{
    if (select_view(diagnostic) != HAL_OK)
        return HAL_ERROR;
    if (diagnostic)
    {
        if (show_diagnostic(r) != HAL_OK)
            return HAL_ERROR;
    }
    else
    {
        if (show_normal(r) != HAL_OK)
            return HAL_ERROR;
    }

    /* 这两个控件始终显示本路Vpp，S1出问题不会连带隐藏正常的S2。 */
    if (show_value("t9", r->vpp_v[0], 1000, "mV", voltage_flags(r, 0)) != HAL_OK)
        return HAL_ERROR;
    return show_value("t11", r->vpp_v[1], 1000, "mV", voltage_flags(r, 1));
}
