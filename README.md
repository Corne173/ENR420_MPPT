# ENR420 MPPT Circuit Implementation Guide

This repository supports the ENR420 MPPT circuit implementation project. It brings together the student guide, STM32 reference material, and the MPPT firmware scaffold used to configure and program the STM32 NUCLEO-F303K8 controller.

> The firmware in this repository is not a finished MPPT solution. It provides the project structure, peripheral setup, measurement pipeline, state machine, telemetry, and safety scaffold. Students still need to implement and validate the actual MPPT control algorithm.

## Start Here

1. Read the [MPPT Circuit Student Implementation Guide](ENR420%20-%20MPPT%20Implemetation%20Guide.pdf).
2. Open the [MPPT firmware](MPPT%20firmware/) project in STM32CubeIDE.
3. Review [`MPPT firmware/Core/Src/main.c`](MPPT%20firmware/Core/Src/main.c), especially the MPPT hook linked below.
4. Build and flash the firmware to the STM32 NUCLEO-F303K8.
5. Test the converter carefully within the operating limits described in the guide.
6. Use GitHub Issues for problems, unclear instructions, firmware questions, or documentation gaps.

## Included Resources

| Resource | Description |
| --- | --- |
| [MPPT Circuit Student Implementation Guide](ENR420%20-%20MPPT%20Implemetation%20Guide.pdf) | Main PDF guide for the project. Start here. |
| [MPPT firmware](MPPT%20firmware/) | STM32CubeIDE firmware project for the MPPT controller. |
| [STM32F reference manual chapters](Manual%20-%20STM32F/) | Split STM32F303 reference-manual sections for easier lookup. |
| [Full STM32 reference manual PDF](rm0316-stm32f303xbcde-stm32f303x68-stm32f328x8-stm32f358xc-stm32f398xe-advanced-armbased-mcus-stmicroelectronics.pdf) | Complete ST RM0316 reference manual. |
| [Serial monitor GUI](mppt_serial_gui.py) | Optional Python helper for monitoring serial data from the controller. |

## Repository Layout

```text
.
|-- ENR420 - MPPT Implemetation Guide.pdf   # Compiled student guide
|-- MPPT firmware/                          # STM32CubeIDE firmware project
|-- Manual - STM32F/                        # STM32 reference manual split by section
|-- rm0316-...-stmicroelectronics.pdf       # Full STM32F reference manual
|-- mppt_serial_gui.py                      # Optional serial monitoring GUI
|-- requirements-mppt-gui.txt               # Python dependency list for the GUI
|-- .gitignore
`-- README.md
```

The LaTeX source and figure assets used to create the guide are local authoring files and are intentionally not part of the GitHub package. The compiled PDF guide is the student-facing document.

## MPPT Firmware

The firmware project is located in [`MPPT firmware/`](MPPT%20firmware/). It includes the STM32CubeIDE project files, CubeMX configuration, source code, HAL drivers, and linker script.

| Path | Purpose |
| --- | --- |
| [`MPPT firmware/Code.ioc`](MPPT%20firmware/Code.ioc) | CubeMX peripheral configuration. |
| [`MPPT firmware/Core/Src/main.c`](MPPT%20firmware/Core/Src/main.c) | Main application entry point and MPPT control scaffold. |
| [`MPPT firmware/Core/Inc/`](MPPT%20firmware/Core/Inc/) | Application header files. |
| [`MPPT firmware/Drivers/`](MPPT%20firmware/Drivers/) | CMSIS and STM32 HAL driver files. |
| [`MPPT firmware/STM32F303K8TX_FLASH.ld`](MPPT%20firmware/STM32F303K8TX_FLASH.ld) | Linker script for the STM32F303K8 target. |

## Firmware Implementation Status

In this section, "implemented" means the functionality is present in the current codebase. It does not mean the converter has been fully bench-tested across all operating conditions.

### What has been implemented

| Area | Current status |
| --- | --- |
| Project setup | STM32CubeIDE/CubeMX project for the STM32F303K8 target. |
| Peripheral setup | ADC1, ADC2, GPIO, TIM1 PWM, TIM3, USART1, and USART2 are configured. |
| Control structure | `main.c` contains a cooperative main loop and MPPT state machine with `INIT`, `IDLE`, `STARTUP`, `RUN`, and `FAULT` states. |
| Measurements | ADC channels are polled and converted from raw counts to voltage, current, and PV power estimates. Current-sensor zero offsets are captured during initialization. |
| Serial interface | The firmware provides start, stop, fault reset, status, and help commands over the serial console. |
| Telemetry | Raw ADC telemetry packets are transmitted for debugging measurements and controller state. |
| PWM and safety scaffold | TIM1 complementary PWM duty commands, gate-driver disable control, duty clamping, startup duty, ADC saturation checking, and latched fault handling are present. |
| Irradiance sensor scaffold | `IrradianceSensor_Task()` is called from the main loop and contains a disabled RS485/Modbus polling skeleton with request building, CRC checking, RS485 direction control, and response parsing placeholders. |

### What still needs to be done

| Work item | Where to work |
| --- | --- |
| Implement the real MPPT algorithm using PV voltage, PV current, and PV power to update the buck and boost duty commands. | [`Mppt_CalculateDuty()` in `main.c`](MPPT%20firmware/Core/Src/main.c#L559) |
| Validate ADC channel mapping, sensor scaling factors, current offsets, and calculated engineering units against real measurements. | [`Measurements_Update()` in `main.c`](MPPT%20firmware/Core/Src/main.c#L597) |
| Verify duty limits, PWM polarity, complementary outputs, gate-driver enable/disable behaviour, and oscilloscope waveforms before energising the converter. | [`PowerStage_SetDuty()` in `main.c`](MPPT%20firmware/Core/Src/main.c#L812) |
| Add practical operating-limit protection. The present check catches saturated ADC readings only; real over-voltage, over-current, and laboratory safety thresholds still need to be added. | [`Measurements_AreInAdcRange()` in `main.c`](MPPT%20firmware/Core/Src/main.c#L633) |
| Complete the irradiance sensor / RS485 Modbus request-response logic. Confirm the sensor slave address, register address, register count, response format, and scaling, then enable and complete the polling scaffold. | [`IrradianceSensor_Task()` in `main.c`](MPPT%20firmware/Core/Src/main.c#L1138) and [`IrradianceSensor_ParseReadResponse()` in `main.c`](MPPT%20firmware/Core/Src/main.c#L1246) |

## Optional Serial GUI

The Python serial monitor can be used to inspect controller output while testing.

Install the dependency:

```bash
pip install -r requirements-mppt-gui.txt
```

Run the GUI:

```bash
python mppt_serial_gui.py
```

## Getting Help

Please use the GitHub **Issues** feature for project problems, bugs, unclear instructions, firmware questions, or documentation gaps. Issues keep questions and solutions visible, so one student's problem can help the next group too.

To create an issue:

1. Open this repository on GitHub.
2. Select the **Issues** tab near the top of the repository page.
3. Search existing issues first to see whether the same problem has already been reported.
4. Select **New issue** and write a clear title and description.
5. Submit the issue and watch the thread for replies or follow-up questions.

GitHub's own guide is here: [Creating an issue](https://docs.github.com/en/issues/tracking-your-work-with-issues/using-issues/creating-an-issue).

When creating the issue, include what you were trying to do, what happened, what you expected, and enough detail for someone else to reproduce the problem. For firmware issues, include the current state, fault code, serial command used, any `ADC,...` telemetry line, relevant code changes, and the hardware setup.

Do not post private information, student numbers, passwords, or anything unrelated to the project.

## Notes for Contributors

- Keep student-facing explanations clear and practical.
- Keep firmware changes documented in the guide when they affect setup, testing, or expected behaviour.
- Keep the compiled PDF guide up to date when local authoring files change.
- Prefer issue discussions for recurring questions so that fixes and explanations remain visible.
- Avoid committing unnecessary generated, temporary, or local authoring files.
