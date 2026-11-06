#include "main.h"
#include "app.h"
#include "capture.h"
#include "hmi.h"
#include "ad9833.h"
#include "meter_config.h"
#include <string.h>

static MeterCalibration calibration;
static MeterResult result;
static uint8_t busy, requested;
static uint32_t last_started;

static void start_measurement(void)
{
    last_started = HAL_GetTick();
    requested = 0;
    busy = 1; /* 内部参考的读取也属于本轮测量，先标忙再清空结果。 */
    /* 新帧失败或提前结束时，不能把上一帧未更新的字段当作本轮结果。 */
    memset(&result, 0, sizeof(result));
    if (Capture_Begin() == HAL_OK)
    {
        /* 与本帧配套的参考电压；不再把VDDA固定当成3.300V。 */
        calibration.vdda_v = Capture_ReferenceVoltage();
    }
    else
    {
        busy = 0;
        result.flags = MEAS_FLAG_CAPTURE;
        Hmi_Show(&result);
    }
}

void App_Init(void)
{
    HAL_Delay(1500); /* 等待屏幕上电，避免第一条命令过早发出。 */
    Hmi_Init();
    if (Capture_Init() != HAL_OK)
        Error_Handler();
    AD9833_Init();
    if (AD9833_SetOutput(METER_DDS_HZ, METER_DDS_WAVE) != HAL_OK)
        Error_Handler();

    busy = 0;
    requested = 1; /* 首帧直接开始，以后自动更新，也可按屏幕按钮提前测量。 */
}

void App_Poll(void)
{
    CaptureState state;
    if (Hmi_TakeRequest())
        requested = 1;

    /* 忙时只记住一次请求，等本帧计算完再开始，防止覆盖正在使用的样本。 */
    if (!busy && (requested || HAL_GetTick() - last_started >= METER_UPDATE_PERIOD_MS))
        start_measurement();
    if (!busy)
        return;

    state = Capture_Poll();
    if (state == CAPTURE_PENDING)
        return;

    if (state == CAPTURE_DONE)
        Signal_Analyze(Capture_Data(), METER_SAMPLE_COUNT, Capture_SampleRate(), &calibration, &result);
    else
        result.flags = MEAS_FLAG_CAPTURE;

    Hmi_Show(&result);
    busy = 0;
}
