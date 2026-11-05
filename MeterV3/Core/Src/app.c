#include "app.h"
#include "capture.h"
#include "hmi.h"
#include "ad9833.h"
#include "meter_config.h"
#include "iwdg.h"
#include <string.h>

static const MeterCalibration calibration = {
    METER_VDDA_V,
    {METER_GAIN_S1, METER_GAIN_S2},
    {METER_OFFSET_S1_V, METER_OFFSET_S2_V}
};
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

static uint8_t key_pressed(void)
{
    uint8_t key = HAL_GPIO_ReadPin(MEASURE_KEY_GPIO_Port, MEASURE_KEY_Pin) == GPIO_PIN_SET;
    if (key != key_previous)
    {
        key_previous = key;
        key_changed = HAL_GetTick();
    }
    /* 电平稳定20ms才认作按下，避免一次按键抖动触发多次。 */
    if (HAL_GetTick() - key_changed >= 20U && key != key_stable)
    {
        key_stable = key;
        if (key == 0U)
            return 1;
    }
    return 0;
}

static void start_measurement(void)
{
    started = HAL_GetTick();
    last_started = started;
    refresh_requested = 0;
    busy = 1;

    /* 防止本轮采集失败或算法提前退出时，未更新字段还沿用上轮结果。 */
    memset(&g_last_result, 0, sizeof(g_last_result));
    if (Capture_Begin() != HAL_OK)
    {
        g_last_result.flags = MEAS_FLAG_CAPTURE;
        finish_measurement();
    }
}

void App_Init(void)
{
    busy = 0;
    diagnostic = 0;
    key_previous = key_stable = 1;
    memset(&g_last_result, 0, sizeof(g_last_result));
    g_last_result.flags = MEAS_FLAG_CAPTURE; /* 上电尚未得到有效样本，不能把0当读数。 */

    HAL_Delay(1500); /* 给屏幕启动时间，避免第一条命令在屏幕未就绪时丢失。 */
    if (Hmi_Init() != HAL_OK)
        ++g_comm_errors;
    if (Capture_Init() != HAL_OK)
        Error_Handler();
    if (AD9833_Init() != HAL_OK)
        Error_Handler();
    /* 频率和波形只在这里设置一次，修改meter_config.h即可。 */
    if (AD9833_SetOutput(METER_DDS_HZ, METER_DDS_WAVE) != HAL_OK)
        Error_Handler();

    key_changed = last_started = HAL_GetTick();
    refresh_requested = 1; /* 首帧立即启动，不让用户上电后还要按一次按钮。 */
}

void App_Poll(void)
{
    HmiRequest request = Hmi_TakeRequest();
    CaptureState capture;
    if (HAL_IWDG_Refresh(&hiwdg) != HAL_OK)
        Error_Handler();

    if (key_pressed())
        request = HMI_REQUEST_MEASURE;
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

    /* 空闲才开新帧。忙时收到的测量请求保留到本帧结束，避免覆盖样本。 */
    if (!busy)
    {
        if (refresh_requested || HAL_GetTick() - last_started >= METER_UPDATE_PERIOD_MS)
            start_measurement();
    }
    if (!busy)
        return;

    capture = Capture_Poll();
    if (capture == CAPTURE_PENDING)
        return; /* 没采满就先处理别的事，下次主循环再来查询。 */

    if (capture == CAPTURE_DONE)
    {
        Signal_Analyze(Capture_Data(), METER_SAMPLE_COUNT, Capture_SampleRate(),
                       &calibration, &g_last_result);
    }
    else
    {
        g_last_result.flags = MEAS_FLAG_CAPTURE;
    }
    finish_measurement();
}
