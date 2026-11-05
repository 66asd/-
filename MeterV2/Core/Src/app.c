/* Foreground application: periodic capture -> analyze -> display.
 * DMA/UART callbacks publish flags/bytes only. One capture runs at a time.
 * Manual requests coalesce into one pending refresh; automatic updates never stop.
 */
#include "app.h"
#include "capture.h"
#include "hmi.h"
#include "ad9833.h"
#include "meter_config.h"
#include "iwdg.h"
#include <string.h>

static const MeterCalibration calibration = {
    METER_VDDA_V, {METER_GAIN_S1, METER_GAIN_S2}, {METER_OFFSET_S1_V, METER_OFFSET_S2_V}};
MeterResult g_last_result;
volatile uint32_t g_comm_errors;
static uint8_t busy, diagnostic, refresh_requested;
static uint8_t key_previous = 1, key_stable = 1;
static uint32_t key_changed, started, last_started;

static void display_result(void)
{
    if (Hmi_Show(&g_last_result, diagnostic) != HAL_OK)
        ++g_comm_errors;
}

static void finish_measurement(void)
{
    display_result();
    g_last_result.elapsed_ms = HAL_GetTick() - started;
    busy = 0;
}

void App_Init(void)
{
    busy = diagnostic = 0;
    key_previous = key_stable = 1;
    memset(&g_last_result, 0, sizeof(g_last_result));
    g_last_result.flags = MEAS_FLAG_TIMEOUT;
    HAL_Delay(1500);
    if (Hmi_Init() != HAL_OK)
        ++g_comm_errors;
    if (Capture_Init() != HAL_OK || AD9833_Init() != HAL_OK)
        Error_Handler();
    if (AD9833_SetOutput(METER_DDS_HZ, AD9833_WAVE_SINE) != HAL_OK)//在这改波形
        Error_Handler();
    key_changed = last_started = HAL_GetTick();
    refresh_requested = 1; /* First sample starts on the first main-loop poll. */
}

void App_Poll(void)
{
    HmiRequest request = Hmi_TakeRequest();
    uint8_t key = HAL_GPIO_ReadPin(MEASURE_KEY_GPIO_Port, MEASURE_KEY_Pin) == GPIO_PIN_SET;
    CaptureState capture;
    if (HAL_IWDG_Refresh(&hiwdg) != HAL_OK)
        Error_Handler();

    if (key != key_previous)
    {
        key_previous = key;
        key_changed = HAL_GetTick();
    }
    if (HAL_GetTick() - key_changed >= 20U && key != key_stable)
    {
        key_stable = key;
        if (!key)
            request = HMI_REQUEST_MEASURE;
    }
    if (request == HMI_REQUEST_DIAGNOSTIC)
    {
        diagnostic = !diagnostic;
        if (!busy)
            display_result();
    }
    if (request == HMI_REQUEST_MEASURE)
    {
        diagnostic = 0;
        refresh_requested = 1;
    }

    /* Unsigned subtraction remains valid across HAL tick rollover. A slow or
     * failed capture never overlaps the next one; the next poll retries when due.
     * Keep the previous screen values visible while the new frame is acquired.
     */
    if (!busy && (refresh_requested || HAL_GetTick() - last_started >= METER_UPDATE_PERIOD_MS))
    {
        started = last_started = HAL_GetTick();
        refresh_requested = 0;
        busy = 1;
        memset(&g_last_result, 0, sizeof(g_last_result));
        if (Capture_Begin() != HAL_OK)
        {
            g_last_result.flags = MEAS_FLAG_TIMEOUT;
            finish_measurement();
            return;
        }
    }
    if (!busy)
        return;
    capture = Capture_Poll();
    if (capture == CAPTURE_PENDING)
        return;
    if (capture == CAPTURE_DONE)
        Signal_Analyze(Capture_Data(), METER_SAMPLE_COUNT, Capture_SampleRate(), &calibration,
                       &g_last_result);
    else
        g_last_result.flags = MEAS_FLAG_TIMEOUT;
    finish_measurement();
}
