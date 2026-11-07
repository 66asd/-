#include "main.h"
#include "app.h"
#include "capture.h"
#include "hmi.h"
#include "ad9833.h"
#include "meter_config.h"
#include <string.h>

/* V4保存本帧VDDA（V），算法里再除4096求每码电压。 */
static MeterCalibration calibration = {METER_VDDA_V};
static MeterResult result; /* 本轮两路RMS、频率、相位、Vpp及错误标志；交给屏幕显示。 */
/* busy=1：正在采集/处理；requested=1：记住一次待执行测量请求。 */
static uint8_t busy, requested;
static uint32_t last_started; /* 最近一次开始测量的毫秒时刻，用于250ms自动刷新调度。 */

/* 开始一轮：记时、清旧结果、启动采样；失败则立即显示错误，不沿用旧数值。 */
static void start_measurement(void)
{
    last_started = HAL_GetTick();
    requested = 0;
    busy = 1; /* 先标忙，再清空本轮结果。 */
    /* 新帧失败或提前结束时，不能把上一帧未更新的字段当作本轮结果。 */
    memset(&result, 0, sizeof(result));
    if (Capture_Begin() != HAL_OK)
    {
        busy = 0;
        result.flags = MEAS_FLAG_CAPTURE;
        Hmi_Show(&result);
    }
}

/* 上电准备：等屏幕、校准ADC、设置DDS；requested=1使首轮自动执行。 */
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

/* 主循环反复调用：接请求 -> 启动 -> 等DMA完成 -> 算结果 -> 发屏幕；等待DMA时先返回。 */
void App_Poll(void)
{
    CaptureState state; /* 本次查询到的采集状态；与文件内表示整轮忙闲的busy不同。 */
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
    {
        calibration.vdda_v = Capture_ReferenceVoltage(); /* 每帧使用实测量程，不再手改VDDA。 */
        Signal_Analyze(Capture_Data(), METER_SAMPLE_COUNT, Capture_SampleRate(), &calibration, &result);
    }
    else
        result.flags = MEAS_FLAG_CAPTURE;

    Hmi_Show(&result);
    busy = 0; /* 数据已计算并发送，本轮结束，之后才允许下一帧覆盖采样数组。 */
}
