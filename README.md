# ENR420 MPPT Circuit Implementation Guide

This repository contains the ENR420 student guide, a preconfigured STM32CubeIDE project, the MPPT firmware, a serial-monitoring GUI, and supporting reference material for the STM32 NUCLEO-F303K8 controller.

> The supplied code includes the peripheral configuration, measurement pipeline, P&O MPPT controller, state machine, temperature and irradiance sensing, telemetry, and basic fault handling. It still requires careful laboratory validation, sensor calibration, and project-specific operating-limit decisions. Practical over-voltage, over-current, and over-temperature thresholds are not supplied.

## Start Here

1. On GitHub, select **Code → Download ZIP**, then extract the ZIP to a normal working folder.
2. Install [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html).
3. In STM32CubeIDE, select **File → Import → General → Existing Projects into Workspace**. Set the root directory to the extracted `MPPT firmware` folder, select the project named `Code`, and finish the import. Import the root project, not the nested `MPPT firmware/STM32CubeIDE` folder.
4. Build `Code`, connect the NUCLEO board through its ST-LINK USB connector with the converter power stage unenergised, launch the supplied `Code Debug` configuration, and select **Resume** after programming stops at `main()`.
5. From the repository root, run `python -m pip install -r requirements-mppt-gui.txt`, then `python mppt_serial_gui.py`. Select the ST-LINK virtual COM port at 115200 baud and connect.
6. Read the [MPPT Circuit Student Implementation Guide](ENR420%20-%20MPPT%20Implemetation%20Guide.pdf) before wiring sensors or energising the converter.

## Included Resources

| Resource | Description |
| --- | --- |
| [MPPT Circuit Student Implementation Guide](ENR420%20-%20MPPT%20Implemetation%20Guide.pdf) | Main PDF guide for the project. Start here. |
| [MPPT firmware](MPPT%20firmware/) | STM32CubeIDE firmware project for the MPPT controller. |
| [STM32F reference manual chapters](Manual%20-%20STM32F/) | Split STM32F303 reference-manual sections for easier lookup. |
| [Serial monitor GUI](mppt_serial_gui.py) | Optional Python helper for monitoring serial data from the controller. |
| [Four-switch buck-boost PSIM schematic](Four_switch_buck_boost.psimsch) | Optional PSIM schematic for simulating the converter and exploring its operating principles. |

## Sensor Wiring at a Glance

Sensor connectors are low-voltage signal connections. They are separate from the high-power **Solar panels in** and output/load terminals; never land a sensor wire on a PV power terminal.

| Sensor | Wire colour | PCB connector and input |
| --- | --- | --- |
| DS18B20 temperature | Red | J8 `3V3` |
| DS18B20 temperature | Yellow | J8 `DQ` |
| DS18B20 temperature | Black | J8 `SGND` |
| SEN0640 irradiance | Brown | J6 `5V` |
| SEN0640 irradiance | Yellow | J6 `A` |
| SEN0640 irradiance | Blue | J6 `B` |
| SEN0640 irradiance | Black | J6 `SGND` |

All four DS18B20 probes share J8 in parallel. The complete 1-Wire bus needs exactly one 4.7 kΩ pull-up from `DQ` to `3V3`: first verify that R20 is populated with a suitable value, and add one external resistor only if it is absent or unsuitable. The guide explains how to identify and label the discovery-order channels `Temp0`–`Temp3`.

## Further Reading

Students who want more background on four-switch buck-boost converter control strategies may find this review useful:

- Lin, G.; Li, Y.; Zhang, Z. "A Review of Control Strategies for Four-Switch Buck-Boost Converters." *World Electric Vehicle Journal* 2025, 16(6), 315. <https://doi.org/10.3390/wevj16060315>

## Repository Layout

```text
.
|-- ENR420 - MPPT Implemetation Guide.pdf   # Compiled student guide
|-- MPPT firmware/                          # STM32CubeIDE firmware project
|-- Manual - STM32F/                        # STM32 reference manual split by section
|-- Four_switch_buck_boost.psimsch          # Optional PSIM converter simulation schematic
|-- mppt_serial_gui.py                      # Optional serial monitoring GUI
|-- requirements-mppt-gui.txt               # Python dependency list for the GUI
|-- .gitignore
`-- README.md
```

The LaTeX source and figure assets used to create the guide are local authoring files and are intentionally not part of the GitHub package. The compiled PDF guide is the student-facing document.

## MPPT Firmware

The firmware project is located in [`MPPT firmware/`](MPPT%20firmware/). It includes the STM32CubeIDE project files, generated peripheral source, application code, HAL drivers, and linker script.

| Path | Purpose |
| --- | --- |
| [`MPPT firmware/.project`](MPPT%20firmware/.project) | Root STM32CubeIDE project descriptor; the imported project is named `Code`. |
| [`MPPT firmware/Code.ioc`](MPPT%20firmware/Code.ioc) | Legacy CubeMX metadata. It is currently out of sync with the supplied generated source and must not be used to regenerate the project. |
| [`MPPT firmware/Core/Src/main.c`](MPPT%20firmware/Core/Src/main.c) | Main application entry point and cooperative state-machine loop. |
| [`MPPT firmware/Core/Src/MpptController.c`](MPPT%20firmware/Core/Src/MpptController.c) | Supplied P&O MPPT controller. |
| [`MPPT firmware/Core/Inc/`](MPPT%20firmware/Core/Inc/) | Application header files. |
| [`MPPT firmware/Drivers/`](MPPT%20firmware/Drivers/) | CMSIS and STM32 HAL driver files. |
| [`MPPT firmware/STM32F303K8TX_FLASH.ld`](MPPT%20firmware/STM32F303K8TX_FLASH.ld) | Linker script for the STM32F303K8 target. |

## Supplied Firmware

The current package provides these code-level capabilities:

| Area | Supplied implementation |
| --- | --- |
| Peripheral setup | ADC1/ADC2 synchronized acquisition, DMA, GPIO, TIM1 complementary PWM, timebases, USART1, and USART2 are configured in the generated C source. |
| Control | The cooperative `INIT`, `IDLE`, `STARTUP`, `RUN`, and `FAULT` state machine uses the supplied P&O MPPT controller, settling and sampling phases, duty clamping, and controlled startup. |
| Measurements | Raw ADC samples are averaged and converted to current, voltage, and input-power estimates; current-sensor zero offsets are captured during initialization. |
| Temperature | The 1-Wire driver discovers multiple DS18B20 probes. Telemetry publishes four discovery indices as `Temp0`–`Temp3`; an unavailable probe is reported as `NA`. |
| Irradiance and RS-485 | The SEN0640 is polled over USART1 using the board's [ST4E1216IDT transceiver](https://www.st.com/resource/en/datasheet/st4e1216.pdf). Request framing, Modbus CRC, big-endian register parsing, half-duplex direction control, and non-blocking transaction handling are implemented. |
| PC console | USART2 provides `s` start, `x` stop, `r` fault reset, `?` status, and `h` help commands through the ST-LINK virtual COM port. |
| Telemetry and GUI | Fixed-position CSV telemetry, live measurements, four temperature traces, irradiance, MPPT status, and optional PC-side CSV recording are supplied. |
| Basic safe-state handling | Gate-driver disable control, PWM duty clamps, ADC acquisition/saturation faults, and latched fault handling are present. |

These statements describe the source code, not a completed hardware qualification. Before energising the converter, verify sensor scaling, PWM routing and polarity, dead time, disable behaviour, and waveforms on the actual PCB. Select approved practical voltage, current, temperature, and communication limits for the laboratory setup; the supplied firmware does not claim those engineering-unit protections.

STM32CubeIDE is used to edit, build, flash, and debug this package. Standalone [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html) is not needed for the supplied project, but it is useful when configuring a new STM32 project from scratch. Do **not** regenerate this project from `Code.ioc`; the current generated C source is the configuration authority.

## Optional Serial GUI

The Python serial monitor can be used to inspect controller output while testing.

Install the dependency from the repository root:

```bash
python -m pip install -r requirements-mppt-gui.txt
```

Run the GUI:

```bash
python mppt_serial_gui.py
```

Select the ST-LINK virtual COM port, leave the baud rate at 115200, and select **Connect**. Check **Status** before **Start**. Select **Start Recording** to choose a CSV
file and begin logging telemetry. The button changes to **Stop Recording** while
logging. Every telemetry packet is written and flushed to disk immediately, so
the GUI does not keep the complete recording in memory. Recording stops cleanly
when the button is selected again, the serial connection closes, or the GUI exits.

The CSV contains the PC receive time followed by the telemetry fields. New columns are appended so existing column positions remain unchanged:
`unix_ms,i_in_raw,i_out_raw,v_out_raw,v_in_raw,valid,temp0_c,temp1_c,irr_w_m2,state,fault,mppt_phase,mppt_gain,mppt_step,buck_duty,boost_duty,reference_power_w,sampled_power_w,temp2_c,temp3_c`.
Status and help messages shown in the terminal are not included.

## Getting Help

Please use the GitHub **Issues** feature for project problems, bugs, unclear instructions, firmware questions, or documentation gaps. Issues keep questions and solutions visible, so one student's problem can help the next group too.

To create an issue:

1. Open this repository on GitHub.
2. Select the **Issues** tab near the top of the repository page.
3. Search existing issues first to see whether the same problem has already been reported.
4. Select **New issue** and write a clear title and description.
5. Submit the issue and watch the thread for replies or follow-up questions.

GitHub's own guide is here: [Creating an issue](https://docs.github.com/en/issues/tracking-your-work-with-issues/using-issues/creating-an-issue).

When creating the issue, include what you were trying to do, what happened, what you expected, and enough detail for someone else to reproduce the problem. For firmware issues, include the current state, fault code, serial command used, a telemetry line, relevant code changes, and the hardware setup.

Do not post private information, student numbers, passwords, or anything unrelated to the project.

## Notes for Contributors

- Keep student-facing explanations clear and practical.
- Keep firmware changes documented in the guide when they affect setup, testing, or expected behaviour.
- Keep the compiled PDF guide up to date when local authoring files change.
- Prefer issue discussions for recurring questions so that fixes and explanations remain visible.
- Avoid committing unnecessary generated, temporary, or local authoring files.
