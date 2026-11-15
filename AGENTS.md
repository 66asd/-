# Project instructions

- Project: STM32F103C8T6 dual ADC oscilloscope, adaptive 5 to 500 kS/s per channel, nominal input 50 Hz to 120 kHz (no 120 kHz software cutoff).
- Do not modify any HMI file. Use the current `DualDDS2.HMI` page3 definitions.
- Keep acquisition, frequency, AC RMS, VPP, waveform and HMI protocol in separate modules.
- Preserve CubeMX USER CODE sections and add application sources to the Keil project.

## Approved code backup scope

- Destination: https://github.com/66asd/-.git (remote `codex-backup`).
- Stable project ID: `stm32f103-adc1-3a77f2a226`.
- Include: `AGENTS.md`, `.gitignore`, `ADC/ADC.ioc`, `ADC/Core/`, `ADC/App/`,
  `ADC/Tests/`, `ADC/Tools/`, `ADC/README.md`, `ADC/hmi_page3_contract.json`, `ADC/MDK-ARM/ADC.uvprojx`,
  `ADC/MDK-ARM/startup_stm32f103xb.s` and reviewed project build configuration.
- Exclude HMI binaries, vendor `ADC/Drivers/` dependencies, IDE session state,
  compiled outputs, credentials and unrelated projects.
- Use the git-code-backup skill before source edits and before completion.
- Git commit identity authorized for this project: 66asd <66asd@users.noreply.github.com>.
