# AD9833 正弦扫频

在 Keil µVision 打开 `MDK-ARM/Sweep.uvprojx`，Rebuild 后用 ST-LINK/SWD 下载。烧录文件为 `MDK-ARM/Sweep/Sweep.hex`。CubeMX 配置为 `Sweep.ioc`，使用 STM32CubeMX 6.16.0 和 STM32CubeF1 1.8.6。业务代码在 `Core/Src/main.c` 的 USER CODE 4 区。CubeMX 关闭自动生成 main 入口，以保留分阶段启动逻辑。

## 接线和观察

| STM32F103C8T6 | AD9833 / 仪器 |
| --- | --- |
| PA4 | FSYNC |
| PA5 | SCLK |
| PA7 | SDATA |
| PA0（ADC1_IN0） | 模块模拟 OUT，与示波器 CH1 并接 |
| GND | 模块 GND 和示波器探头地 |
| PA13、PA14 | ST-LINK 的 SWDIO、SWCLK |

按模块额定电压供电，并确认控制输入兼容 3.3 V。接 PA0 前用示波器直流耦合确认整个波形都在 0 至 VDDA 之间。若模块输出带负压或超过 3.3 V，先偏置或衰减。PA0 断开时仍可在 OUT 观察扫频，悬空 ADC 的读数无意义。

复位后从 100 Hz 开始，每 100 ms 增加 100 Hz。30 kHz 也保持 100 ms，再回到 100 Hz。每轮 300 档，约 30 s。示波器打开上升沿触发与频率读数，低端可用 2 ms/div，高端可用 10 µs/div。跨换档边界的自动频率读数可能混合两档。

Keil Watch 中观察 `sweep_set_hz`（当前设定，Hz）和 `measured_millihz`（本档 PA0 实测，除以 1000 得 Hz）。每次换档清零实测值，等待 3 ms 后启动单帧采集，有效帧完成才更新。弱信号、异常周期和直流帧返回 0。

## 测频与校准

相邻 DDS-codex/MeterV5 的旧算法在均值附近对上升穿越做线性插值，按首末穿越求周期。其最小有效间隔按 20 kHz 设置，30 kHz 输入会跳过交替的穿越，可能显示约 15 kHz。

本工程由 TIM3 触发 ADC1，DMA 每档采 2048 点。100 至 1900 Hz 档用 32 kS/s，可覆盖 100 Hz 的 6.4 个周期。其余档用 500 kS/s，30 kHz 时约 16.7 点/周期、每帧约 123 个周期。对带迟滞的上升穿越做亚采样插值，用全部有效穿越的序号和时间做直线拟合。设定频率只选择采样率，实测值取自 ADC。稳定时间、采集、SPI 发送和启动校准均按状态推进。

默认板上 HSE 为 8 MHz，PLL 将主频升到 72 MHz。AD9833 参考时钟按 25 MHz 计算。模块晶振不同时请改 `DDS_MCLK_HZ`。校准输出时用独立频率计或已校准示波器，在高频档以“当前 MCLK 常量 × 仪器实测频率 / 设定频率”估计实际 MCLK。校准回采时可给 PA0 输入已知频率和合适偏置的独立正弦，分开核对采样时基。

## 验证

Keil Rebuild 后在安装 Unicorn 2 的 Python 环境运行 `python Tests/check_firmware.py`。该测试执行 HEX 中的 ARM 指令，检查所有调谐字、末档回绕、带偏置及噪声的 300 个正弦输入和直流输入。实物精度需在平台稳定段与独立仪器比对。
