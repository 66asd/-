/* TJC page0: HMI owns normal labels t0..t3 and button b0; MCU updates t4..t7 and S1/S2 Vpp in
 * t9/t11, with unit suffixes. TX is bounded foreground HAL transmit. RX is a single
 * producer/consumer ring: ISR owns head, foreground owns tail; byte indices are atomic on
 * Cortex-M3.
 */
#include "hmi.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
static uint8_t rx_byte;
static volatile uint8_t ring[32], head, tail, rx_fault;
static uint8_t parser;
typedef enum
{
    VIEW_UNKNOWN,
    VIEW_NORMAL,
    VIEW_DIAGNOSTIC
} HmiView;
static HmiView current_view = VIEW_UNKNOWN;
volatile uint32_t g_hmi_rx_errors;
static HAL_StatusTypeDef command(const char *s)
{
    uint8_t data[96];
    size_t n = strlen(s);
    if (n > sizeof(data) - 3U)
        return HAL_ERROR;
    memcpy(data, s, n);
    data[n++] = 255;
    data[n++] = 255;
    data[n++] = 255;
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
static void fixed2(char *b, size_t n, float value, float units, const char *suffix)
{
    long v = (long)(value * units * 100.0f + (value < 0 ? -0.5f : 0.5f));
    unsigned long m = (unsigned long)(v < 0 ? -v : v);
    (void)snprintf(b, n, "%s%lu.%02lu%s", v < 0 ? "-" : "", m / 100UL, m % 100UL, suffix);
}
HAL_StatusTypeDef Hmi_Init(void)
{
    head = tail = rx_fault = parser = 0;
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
    uint8_t next;
    if (h->Instance != USART1)
        return;
    next = (uint8_t)((head + 1U) & 31U);
    if (next != tail)
    {
        ring[head] = rx_byte;
        head = next;
    }
    else
    {
        ++g_hmi_rx_errors;
        rx_fault = 1;
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
    uint8_t b, result = 0;
    if (rx_fault)
    {
        (void)HAL_UART_AbortReceive(&huart1);
        tail = head;
        parser = 0;
        rx_fault = 0; /* clear before rearming so a new ISR fault is never lost */
        if (HAL_UART_Receive_IT(&huart1, &rx_byte, 1) != HAL_OK)
            rx_fault = 1;
    }
    while (tail != head)
    {
        b = ring[tail];
        tail = (uint8_t)((tail + 1U) & 31U);
        if (parser == 0)
        {
            if (b == 0xA5)
                parser = 1;
        }
        else if (parser == 1)
        {
            if (b == 1 || b == 2)
                parser = (uint8_t)(b + 1U);
            else
                parser = (b == 0xA5) ? 1 : 0;
        }
        else
        {
            if (b == 0x5A)
                result = (uint8_t)(parser - 1U);
            parser = (b == 0xA5) ? 1 : 0;
        }
    }
    return (HmiRequest)result;
}
/* Normal text belongs to the HMI txt properties/page events. Only a view
 * transition may reload page0, restoring its compiled labels after diagnostics.
 * Ordinary refreshes never reload the page or overwrite its fixed captions.
 */
static HAL_StatusTypeDef select_view(uint8_t diagnostic)
{
    const char *names[] = {"t0", "t1", "t2", "t3"};
    const char *debug[] = {"S1 DC/mV", "S1 ALL", "S1 Vpp", "S2 DC/mV"};
    unsigned i;
    if (diagnostic)
    {
        if (current_view == VIEW_DIAGNOSTIC)
            return HAL_OK;
        current_view = VIEW_UNKNOWN; /* retry all captions after a partial TX failure */
        for (i = 0; i < 4; ++i)
            if (text(names[i], debug[i]) != HAL_OK)
                return HAL_ERROR;
        current_view = VIEW_DIAGNOSTIC;
    }
    else if (current_view != VIEW_NORMAL)
    {
        if (command("page 0") != HAL_OK)
            return HAL_ERROR;
        current_view = VIEW_NORMAL;
    }
    return HAL_OK;
}
/* Keep bad samples distinguishable from the HMI's startup placeholder.
 * These strings fit the existing text controls and never masquerade as readings.
 */
static const char *invalid_reason(uint32_t flags, uint8_t frequency_field)
{
    if (flags & MEAS_FLAG_CAL)
        return "CAL ERR";
    if (flags & MEAS_FLAG_TIMEOUT)
        return "ADC ERR";
    if (frequency_field && (flags & MEAS_FLAG_FREQ))
        return "NO FREQ";
    if (flags & MEAS_FLAG_CLIP)
        return "CLIP";
    if (flags & MEAS_FLAG_LOW)
        return "LOW SIG";
    if (flags & MEAS_FLAG_FREQ)
        return "NO FREQ";
    return "INVALID";
}
HAL_StatusTypeDef Hmi_Show(const MeterResult *r, uint8_t diag)
{
    char b[24];
    unsigned i;
    float v, scale;
    uint32_t invalid;
    const char *unit;
    const char *values[] = {"t4", "t5", "t6", "t7", "t9", "t11"};
    if (select_view(diag) != HAL_OK)
        return HAL_ERROR;
    for (i = 0; i < sizeof(values) / sizeof(values[0]); ++i)
    {
        invalid = r->flags & (MEAS_FLAG_TIMEOUT | MEAS_FLAG_CAL);
        scale = 1000.0f;
        unit = "mV";
        if (i >= 4U)
        {
            /* t9=S1 Vpp, t11=S2 Vpp in either view. Each channel has its own
             * validity: a clipped S1 must not hide an otherwise valid S2.
             */
            unsigned channel = i - 4U;
            v = r->vpp_v[channel];
            invalid |= r->channel_flags[channel] & (MEAS_FLAG_LOW | MEAS_FLAG_CLIP);
        }
        else if (diag)
        {
            v = (i == 0)   ? r->dc_v[0]
                : (i == 1) ? r->total_rms_v[0]
                : (i == 2) ? r->vpp_v[0]
                           : r->dc_v[1];
        }
        else
        {
            v = (i < 2) ? r->ac_rms_v[i] : (i == 2) ? r->frequency_hz[0] : r->phase_deg;
            if (i < 2)
                invalid |= r->channel_flags[i] & (MEAS_FLAG_LOW | MEAS_FLAG_CLIP);
            else
            {
                scale = 1.0f;
                unit = (i == 2U) ? "Hz" : "deg"; /* ASCII works with the GB2312 screen font. */
                if (i == 2U)
                    invalid |= r->channel_flags[0];
                else
                    invalid |= r->flags;
            }
        }
        if (invalid)
            strcpy(b, invalid_reason(invalid, (uint8_t)(!diag && i == 2U)));
        else
            fixed2(b, sizeof(b), v, scale, unit);
        if (text(values[i], b) != HAL_OK)
            return HAL_ERROR;
    }
    return HAL_OK;
}
