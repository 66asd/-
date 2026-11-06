#include "hmi.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

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


static uint8_t rx_byte;
static volatile uint8_t parser, rx_fault, requested;

void Hmi_Init(void)
{
    parser = requested = rx_fault = 0;
    if (HAL_UART_Receive_IT(&huart1, &rx_byte, 1) != HAL_OK)
        rx_fault = 1;
    command("page 0");
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *h)
{
    if (h->Instance != USART1)
        return;

    /* 只接受按钮命令A5 01 5A；这里只保存请求，采样留给主循环。 */
    if (parser == 0)
    {
        if (rx_byte == 0xA5)
            parser = 1;
    }
    else if (parser == 1)
    {
        if (rx_byte == 0x01)
            parser = 2;
        else if (rx_byte != 0xA5)
            parser = 0;
    }
    else
    {
        if (rx_byte == 0x5A)
            requested = 1;
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
        rx_fault = 1;
}

uint8_t Hmi_TakeRequest(void)
{
    uint8_t request;
    uint32_t mask;
    if (rx_fault)
    {
        /* 串口出错后重新接收，避免一次接触不良使按钮以后一直失效。 */
        HAL_UART_AbortReceive(&huart1);
        parser = requested = rx_fault = 0;
        if (HAL_UART_Receive_IT(&huart1, &rx_byte, 1) != HAL_OK)
            rx_fault = 1;
    }
    /* 取走与清空一起完成，避免清空时抹掉中断刚收到的新请求。 */
    mask = __get_PRIMASK();
    __disable_irq();
    request = requested;
    requested = 0;
    __set_PRIMASK(mask);
    return request;
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


void Hmi_Show(const MeterResult *r)
{
    uint32_t frequency_error = r->channel_flags[0];
    if (frequency_error & MEAS_FLAG_FREQ)
        frequency_error = MEAS_FLAG_FREQ;
    frequency_error |= r->flags & MEAS_FLAG_CAPTURE;

    /* 发送失败就结束本次刷新，下一帧再发；不让屏幕通信阻止后续采样。 */
    if (show_value("t4", r->ac_rms_v[0], 1000, "mV", voltage_flags(r, 0)) != HAL_OK)
        return;
    if (show_value("t5", r->ac_rms_v[1], 1000, "mV", voltage_flags(r, 1)) != HAL_OK)
        return;
    if (show_value("t6", r->frequency_hz[0], 1, "Hz", frequency_error) != HAL_OK)
        return;
    if (show_value("t7", r->phase_deg, 1, "deg", r->flags) != HAL_OK)
        return;
    if (show_value("t9", r->vpp_v[0], 1000, "mV", voltage_flags(r, 0)) != HAL_OK)
        return;
    show_value("t11", r->vpp_v[1], 1000, "mV", voltage_flags(r, 1));
}
