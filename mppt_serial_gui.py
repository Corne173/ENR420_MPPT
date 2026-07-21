"""Minimal serial GUI for the ENR MPPT controller.

Expected firmware telemetry packet:
i_in_raw,i_out_raw,v_out_raw,v_in_raw,valid,temp0_c,temp1_c,irr_w_m2,state,fault,
mppt_phase,mppt_gain,mppt_step,buck_duty,boost_duty,reference_power_w,sampled_power_w

The PC prepends ``unix_ms`` when it receives a telemetry packet. Legacy
``ADC,ms,seq,...`` firmware packets are accepted and displayed in the new
format as well.
"""

from __future__ import annotations

import csv
import queue
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Deque, TextIO

import tkinter as tk
from tkinter import filedialog, messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover - handled at runtime in the GUI
    serial = None
    list_ports = None


BAUD_DEFAULT = 115200
MAX_POINTS = 600
TERMINAL_MAX_LINES = 500
ADC_MAX_COUNTS = 4095.0
ADC_REFERENCE_VOLTAGE = 3.3
INPUT_VOLTAGE_SENSOR_SCALE = 46.45
OUTPUT_VOLTAGE_SENSOR_SCALE = 28.78
INPUT_CURRENT_SENSOR_V_PER_A = 0.100
OUTPUT_CURRENT_SENSOR_V_PER_A = 0.0333
I_IN_OFFSET_COUNTS = 804
I_OUT_OFFSET_COUNTS = 2030
CSV_HEADER = (
    'unix_ms',
    'i_in_raw',
    'i_out_raw',
    'v_out_raw',
    'v_in_raw',
    'valid',
    'temp0_c',
    'temp1_c',
    'irr_w_m2',
    'state',
    'fault',
    'mppt_phase',
    'mppt_gain',
    'mppt_step',
    'buck_duty',
    'boost_duty',
    'reference_power_w',
    'sampled_power_w',
)


@dataclass(frozen=True)
class SerialLine:
    text: str
    received_unix_ms: int


@dataclass(frozen=True)
class TelemetryPacket:
    unix_ms: int
    i_in_raw: int
    i_out_raw: int
    v_out_raw: int
    v_in_raw: int
    valid: bool
    temp0_c: float | None
    temp1_c: float | None
    irr_w_m2: float | None
    state: str
    fault: str
    mppt_phase: str | None = None
    mppt_gain: float | None = None
    mppt_step: float | None = None
    buck_duty: float | None = None
    boost_duty: float | None = None
    reference_power_w: float | None = None
    sampled_power_w: float | None = None


class TelemetryCsvRecorder:
    def __init__(self) -> None:
        self._file: TextIO | None = None
        self._writer = None
        self.path: Path | None = None
        self.row_count = 0

    @property
    def is_recording(self) -> bool:
        return self._file is not None

    def start(self, path: Path) -> None:
        if self.is_recording:
            raise RuntimeError('Telemetry recording is already active')

        csv_path = Path(path)
        csv_file = csv_path.open('w', encoding='utf-8', newline='')
        try:
            writer = csv.writer(csv_file)
            writer.writerow(CSV_HEADER)
            csv_file.flush()
        except Exception:
            csv_file.close()
            raise
        self._file = csv_file
        self._writer = writer
        self.path = csv_path
        self.row_count = 0

    def write(self, packet: TelemetryPacket) -> None:
        if self._file is None or self._writer is None:
            return

        self._writer.writerow(
            (
                packet.unix_ms,
                packet.i_in_raw,
                packet.i_out_raw,
                packet.v_out_raw,
                packet.v_in_raw,
                1 if packet.valid else 0,
                packet.temp0_c,
                packet.temp1_c,
                packet.irr_w_m2,
                packet.state,
                packet.fault,
                packet.mppt_phase,
                packet.mppt_gain,
                packet.mppt_step,
                packet.buck_duty,
                packet.boost_duty,
                packet.reference_power_w,
                packet.sampled_power_w,
            )
        )
        self._file.flush()
        self.row_count += 1

    def stop(self) -> None:
        csv_file = self._file
        self._file = None
        self._writer = None
        if csv_file is not None:
            csv_file.close()


def parse_optional_float(value: str) -> float | None:
    value = value.strip()
    if not value or value.upper() == "NA":
        return None
    return float(value)


def format_optional(value: float | None, format_spec: str) -> str:
    if value is None:
        return "-"
    return format_spec.format(value)


def adc_raw_to_volts(raw_count: int) -> float:
    return (raw_count * ADC_REFERENCE_VOLTAGE) / ADC_MAX_COUNTS


def input_current_from_raw(raw_count: int) -> float:
    offset_v = adc_raw_to_volts(I_IN_OFFSET_COUNTS)
    return (adc_raw_to_volts(raw_count) - offset_v) / INPUT_CURRENT_SENSOR_V_PER_A


def output_current_from_raw(raw_count: int) -> float:
    offset_v = adc_raw_to_volts(I_OUT_OFFSET_COUNTS)
    return (adc_raw_to_volts(raw_count) - offset_v) / OUTPUT_CURRENT_SENSOR_V_PER_A


def input_voltage_from_raw(raw_count: int) -> float:
    return adc_raw_to_volts(raw_count) * INPUT_VOLTAGE_SENSOR_SCALE


def output_voltage_from_raw(raw_count: int) -> float:
    return adc_raw_to_volts(raw_count) * OUTPUT_VOLTAGE_SENSOR_SCALE


class SerialReader(threading.Thread):
    def __init__(
        self,
        port: str,
        baudrate: int,
        output_queue: queue.Queue[str | SerialLine],
    ):
        super().__init__(daemon=True)
        self._port_name = port
        self._baudrate = baudrate
        self._output_queue = output_queue
        self._stop_event = threading.Event()
        self.serial_port = None

    def run(self) -> None:
        try:
            self.serial_port = serial.Serial(
                self._port_name,
                self._baudrate,
                timeout=0.1,
                write_timeout=0.2,
            )
            self._output_queue.put(f"__CONNECTED__:{self._port_name}")
        except Exception as exc:
            self._output_queue.put(f"__ERROR__:Could not open {self._port_name}: {exc}")
            return

        while not self._stop_event.is_set():
            try:
                raw_line = self.serial_port.readline()
            except Exception as exc:
                self._output_queue.put(f"__ERROR__:Serial read failed: {exc}")
                break

            if raw_line:
                text = raw_line.decode("utf-8", errors="replace").strip()
                if text:
                    self._output_queue.put(
                        SerialLine(
                            text=text,
                            received_unix_ms=time.time_ns() // 1_000_000,
                        )
                    )

        try:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
        finally:
            self._output_queue.put("__DISCONNECTED__")

    def stop(self) -> None:
        self._stop_event.set()

    def write_byte(self, value: bytes) -> None:
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.write(value)


class MpptSerialGui(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("ENR MPPT Serial Monitor")
        self.minsize(1040, 680)

        self.reader: SerialReader | None = None
        self.serial_queue: queue.Queue[str | SerialLine] = queue.Queue()
        self.recorder = TelemetryCsvRecorder()
        self.terminal_line_count = 0

        self.current_buffers: dict[str, Deque[float | None]] = {
            "i_in_a": deque(maxlen=MAX_POINTS),
            "i_out_a": deque(maxlen=MAX_POINTS),
        }
        self.voltage_buffers: dict[str, Deque[float | None]] = {
            "v_in_v": deque(maxlen=MAX_POINTS),
            "v_out_v": deque(maxlen=MAX_POINTS),
        }
        self.temperature_buffers: dict[str, Deque[float | None]] = {
            "temp0_c": deque(maxlen=MAX_POINTS),
            "temp1_c": deque(maxlen=MAX_POINTS),
        }
        self.irradiance_buffers: dict[str, Deque[float | None]] = {
            "irr_w_m2": deque(maxlen=MAX_POINTS),
        }

        self.connection_status = tk.StringVar(value="Disconnected")
        self.packet_status = tk.StringVar(value="No telemetry packets received")
        self.state_value = tk.StringVar(value="-")
        self.fault_value = tk.StringVar(value="-")
        self.valid_value = tk.StringVar(value="-")
        self.measurement_values = {
            "i_in_a": tk.StringVar(value="-"),
            "i_out_a": tk.StringVar(value="-"),
            "v_out_v": tk.StringVar(value="-"),
            "v_in_v": tk.StringVar(value="-"),
        }
        self.sensor_values = {
            "temp0_c": tk.StringVar(value="-"),
            "temp1_c": tk.StringVar(value="-"),
            "irr_w_m2": tk.StringVar(value="-"),
        }

        self._configure_style()
        self._build_layout()
        self.refresh_ports()

        self.after(50, self._process_serial_queue)
        self.after(100, self._draw_plot)

    def _configure_style(self) -> None:
        style = ttk.Style(self)
        style.configure("TFrame", background="#f5f7fa")
        style.configure("Header.TFrame", background="#17202a")
        style.configure("Header.TLabel", background="#17202a", foreground="#ffffff")
        style.configure("TLabel", background="#f5f7fa", foreground="#1f2933")
        style.configure("Value.TLabel", background="#ffffff", foreground="#111827")
        style.configure("TButton", padding=(10, 6))
        style.configure("Primary.TButton", padding=(12, 7))

    def _build_layout(self) -> None:
        self.configure(background="#f5f7fa")

        header = ttk.Frame(self, style="Header.TFrame")
        header.pack(fill=tk.X)
        ttk.Label(
            header,
            text="ENR MPPT Serial Monitor",
            style="Header.TLabel",
            font=("Segoe UI", 15, "bold"),
        ).pack(side=tk.LEFT, padx=16, pady=12)
        ttk.Label(
            header,
            textvariable=self.connection_status,
            style="Header.TLabel",
            font=("Segoe UI", 10),
        ).pack(side=tk.RIGHT, padx=16)

        controls = ttk.Frame(self)
        controls.pack(fill=tk.X, padx=14, pady=(14, 8))

        ttk.Label(controls, text="COM port").pack(side=tk.LEFT)
        self.port_combo = ttk.Combobox(controls, width=24, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=(6, 10))
        ttk.Button(controls, text="Refresh", command=self.refresh_ports).pack(side=tk.LEFT)

        ttk.Label(controls, text="Baud").pack(side=tk.LEFT, padx=(18, 0))
        self.baud_entry = ttk.Entry(controls, width=10)
        self.baud_entry.insert(0, str(BAUD_DEFAULT))
        self.baud_entry.pack(side=tk.LEFT, padx=(6, 10))

        self.connect_button = ttk.Button(
            controls,
            text="Connect",
            style="Primary.TButton",
            command=self.toggle_connection,
        )
        self.connect_button.pack(side=tk.LEFT, padx=(0, 18))

        ttk.Button(controls, text="Start", command=lambda: self.send_command(b"s")).pack(
            side=tk.LEFT,
            padx=4,
        )
        ttk.Button(controls, text="Stop", command=lambda: self.send_command(b"x")).pack(
            side=tk.LEFT,
            padx=4,
        )
        ttk.Button(controls, text="Reset Fault", command=lambda: self.send_command(b"r")).pack(
            side=tk.LEFT,
            padx=4,
        )
        ttk.Button(controls, text="Status", command=lambda: self.send_command(b"?")).pack(
            side=tk.LEFT,
            padx=4,
        )

        self.record_button = ttk.Button(
            controls,
            text='Start Recording',
            command=self.toggle_recording,
            state=tk.DISABLED,
        )
        self.record_button.pack(side=tk.LEFT, padx=(14, 4))

        main = ttk.Frame(self)
        main.pack(fill=tk.BOTH, expand=True, padx=14, pady=8)
        main.columnconfigure(0, weight=5)
        main.columnconfigure(1, weight=1)
        main.rowconfigure(0, weight=1)

        left = ttk.Frame(main)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        left.rowconfigure(1, weight=1)
        left.columnconfigure(0, weight=1)

        summary = ttk.Frame(left)
        summary.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        for column in range(11):
            summary.columnconfigure(column, weight=1)

        self._add_value(summary, "I in A", self.measurement_values["i_in_a"], 0, numeric=True)
        self._add_value(summary, "I out A", self.measurement_values["i_out_a"], 1, numeric=True)
        self._add_value(summary, "V out V", self.measurement_values["v_out_v"], 2, numeric=True)
        self._add_value(summary, "V in V", self.measurement_values["v_in_v"], 3, numeric=True)
        self._add_value(summary, "Valid", self.valid_value, 4)
        self._add_value(summary, "Temp0 C", self.sensor_values["temp0_c"], 5, numeric=True)
        self._add_value(summary, "Temp1 C", self.sensor_values["temp1_c"], 6, numeric=True)
        self._add_value(summary, "Irr W/m2", self.sensor_values["irr_w_m2"], 7, numeric=True)
        self._add_value(summary, "State", self.state_value, 8)
        self._add_value(summary, "Fault", self.fault_value, 9)

        plot_frame = ttk.Frame(left)
        plot_frame.grid(row=1, column=0, sticky="nsew")
        plot_frame.rowconfigure(0, weight=1)
        plot_frame.rowconfigure(1, weight=1)
        plot_frame.columnconfigure(0, weight=1)
        plot_frame.columnconfigure(1, weight=1)
        self.current_canvas = self._create_plot_canvas(plot_frame)
        self.current_canvas.grid(row=0, column=0, sticky="nsew", padx=(0, 4), pady=(0, 4))
        self.voltage_canvas = self._create_plot_canvas(plot_frame)
        self.voltage_canvas.grid(row=0, column=1, sticky="nsew", padx=(4, 0), pady=(0, 4))
        self.temperature_canvas = self._create_plot_canvas(plot_frame)
        self.temperature_canvas.grid(row=1, column=0, sticky="nsew", padx=(0, 4), pady=(4, 0))
        self.irradiance_canvas = self._create_plot_canvas(plot_frame)
        self.irradiance_canvas.grid(row=1, column=1, sticky="nsew", padx=(4, 0), pady=(4, 0))

        ttk.Label(left, textvariable=self.packet_status).grid(
            row=2,
            column=0,
            sticky="ew",
            pady=(8, 0),
        )

        right = ttk.Frame(main)
        right.grid(row=0, column=1, sticky="nsew", padx=(8, 0))
        right.rowconfigure(1, weight=1)
        right.columnconfigure(0, weight=1)

        ttk.Label(
            right,
            text="Serial terminal (telemetry uses PC Unix ms)",
            font=("Segoe UI", 11, "bold"),
        ).grid(
            row=0,
            column=0,
            sticky="w",
            pady=(0, 6),
        )
        terminal_frame = ttk.Frame(right)
        terminal_frame.grid(row=1, column=0, sticky="nsew")
        terminal_frame.rowconfigure(0, weight=1)
        terminal_frame.columnconfigure(0, weight=1)

        self.terminal = tk.Text(
            terminal_frame,
            width=42,
            height=10,
            wrap=tk.NONE,
            background="#111827",
            foreground="#e5e7eb",
            insertbackground="#ffffff",
            font=("Consolas", 9),
        )
        self.terminal.grid(row=0, column=0, sticky="nsew")
        scroll = ttk.Scrollbar(terminal_frame, orient=tk.VERTICAL, command=self.terminal.yview)
        scroll.grid(row=0, column=1, sticky="ns")
        self.terminal.configure(yscrollcommand=scroll.set)

    def _create_plot_canvas(self, parent: ttk.Frame) -> tk.Canvas:
        return tk.Canvas(
            parent,
            background="#ffffff",
            highlightthickness=1,
            highlightbackground="#d1d5db",
        )

    def _add_value(
        self,
        parent: ttk.Frame,
        label: str,
        variable: tk.StringVar,
        column: int,
        *,
        numeric: bool = False,
    ) -> None:
        frame = ttk.Frame(parent)
        frame.grid(row=0, column=column, sticky="ew", padx=3)
        ttk.Label(frame, text=label, font=("Segoe UI", 8)).pack(anchor="w")
        ttk.Label(
            frame,
            textvariable=variable,
            style="Value.TLabel",
            anchor="center",
            font=(("Consolas", 11, "bold") if numeric else ("Segoe UI", 11, "bold")),
            padding=(8, 6),
        ).pack(fill=tk.X)

    def refresh_ports(self) -> None:
        if list_ports is None:
            self.connection_status.set("pyserial is not installed")
            self.port_combo["values"] = []
            return

        ports = [port.device for port in list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and not self.port_combo.get():
            self.port_combo.set(ports[0])

    def toggle_connection(self) -> None:
        if self.reader is not None:
            self.disconnect()
        else:
            self.connect()

    def connect(self) -> None:
        if serial is None:
            messagebox.showerror(
                "Missing dependency",
                "Install pyserial first:\npython -m pip install pyserial",
            )
            return

        port = self.port_combo.get().strip()
        if not port:
            messagebox.showwarning("No COM port", "Select a COM port first.")
            return

        try:
            baudrate = int(self.baud_entry.get())
        except ValueError:
            messagebox.showwarning("Invalid baud", "Enter a numeric baud rate.")
            return

        self.reader = SerialReader(port, baudrate, self.serial_queue)
        self.reader.start()
        self.connect_button.configure(text="Disconnect")
        self.connection_status.set(f"Connecting to {port}...")

    def disconnect(self) -> None:
        self.stop_recording()
        if self.reader is not None:
            self.reader.stop()
            self.reader = None
        self.record_button.configure(state=tk.DISABLED)
        self.connect_button.configure(text="Connect")
        self.connection_status.set("Disconnected")

    def toggle_recording(self) -> None:
        if self.recorder.is_recording:
            self.stop_recording()
        else:
            self.start_recording()

    def start_recording(self) -> None:
        serial_port = self.reader.serial_port if self.reader is not None else None
        if serial_port is None or not serial_port.is_open:
            messagebox.showwarning('Not connected', 'Connect to a serial port first.')
            return

        filename = filedialog.asksaveasfilename(
            title='Record telemetry to CSV',
            defaultextension='.csv',
            filetypes=(('CSV files', '*.csv'), ('All files', '*.*')),
            initialfile=f'mppt_telemetry_{datetime.now():%Y%m%d_%H%M%S}.csv',
        )
        if not filename:
            return

        try:
            self.recorder.start(Path(filename))
        except (OSError, csv.Error) as exc:
            self._append_terminal(f'[ERROR] Could not start recording: {exc}')
            messagebox.showerror('Recording error', f'Could not start recording:\n{exc}')
            return

        self.record_button.configure(text='Stop Recording')
        self._append_terminal(f'[GUI] Recording telemetry to {self.recorder.path}')

    def stop_recording(self) -> None:
        if not self.recorder.is_recording:
            return

        path = self.recorder.path
        row_count = self.recorder.row_count
        try:
            self.recorder.stop()
        except OSError as exc:
            self._append_terminal(f'[ERROR] Could not close recording {path}: {exc}')
            messagebox.showerror('Recording error', f'Could not close the CSV file:\n{exc}')
        else:
            self._append_terminal(
                f'[GUI] Recording stopped: {row_count} telemetry rows saved to {path}'
            )
        finally:
            self.record_button.configure(text='Start Recording')

    def _handle_recording_error(self, error: Exception) -> None:
        path = self.recorder.path
        try:
            self.recorder.stop()
        except OSError as close_error:
            error = OSError(f'{error}; additionally could not close the file: {close_error}')

        self.record_button.configure(text='Start Recording')
        self._append_terminal(f'[ERROR] Recording stopped for {path}: {error}')
        messagebox.showerror('Recording error', f'Recording stopped because writing failed:\n{error}')

    def send_command(self, command: bytes) -> None:
        if self.reader is None:
            self._append_terminal("[GUI] Not connected")
            return

        try:
            self.reader.write_byte(command)
        except Exception as exc:
            self._append_terminal(f"[GUI] Command failed: {exc}")
            return

        self._append_terminal(f"[GUI] sent {command.decode(errors='replace')}")

    def _process_serial_queue(self) -> None:
        while True:
            try:
                item = self.serial_queue.get_nowait()
            except queue.Empty:
                break

            if isinstance(item, SerialLine):
                packet = self._parse_telemetry_packet(
                    item.text,
                    item.received_unix_ms,
                )
                if packet is not None:
                    if self.recorder.is_recording:
                        try:
                            self.recorder.write(packet)
                        except (OSError, csv.Error) as exc:
                            self._handle_recording_error(exc)
                    self._append_terminal(
                        self._format_telemetry_line(
                            item.text,
                            item.received_unix_ms,
                        )
                    )
                    self._handle_packet(packet)
                else:
                    self._append_terminal(item.text)
                continue

            line = item
            if line.startswith("__CONNECTED__:"):
                port = line.split(":", 1)[1]
                self.connection_status.set(f"Connected to {port}")
                continue

            if line == "__DISCONNECTED__":
                if self.reader is None:
                    self.connection_status.set("Disconnected")
                else:
                    self.connection_status.set("Disconnected")
                    self.reader = None
                    self.connect_button.configure(text="Connect")
                continue

            if line.startswith("__ERROR__:"):
                self._append_terminal(line.replace("__ERROR__:", "[ERROR] ", 1))
                self.connection_status.set("Serial error")
                self.reader = None
                self.connect_button.configure(text="Connect")
                continue

            self._append_terminal(line)

        serial_port = self.reader.serial_port if self.reader is not None else None
        connected = serial_port is not None and serial_port.is_open
        if self.recorder.is_recording and not connected:
            self.stop_recording()
        self.record_button.configure(state=tk.NORMAL if connected else tk.DISABLED)

        self.after(50, self._process_serial_queue)

    def _parse_telemetry_packet(
        self,
        line: str,
        received_unix_ms: int,
    ) -> TelemetryPacket | None:
        parts = line.split(",")

        try:
            if parts[0] == "ADC" and len(parts) >= 13:
                parts = parts[3:]

            if len(parts) >= 10 and parts[0] != "ADC":
                return TelemetryPacket(
                    unix_ms=received_unix_ms,
                    i_in_raw=int(parts[0]),
                    i_out_raw=int(parts[1]),
                    v_out_raw=int(parts[2]),
                    v_in_raw=int(parts[3]),
                    valid=parts[4] == "1",
                    temp0_c=parse_optional_float(parts[5]),
                    temp1_c=parse_optional_float(parts[6]),
                    irr_w_m2=parse_optional_float(parts[7]),
                    state=parts[8],
                    fault=parts[9],
                    mppt_phase=(parts[10].strip() or None) if len(parts) > 10 else None,
                    mppt_gain=parse_optional_float(parts[11]) if len(parts) > 11 else None,
                    mppt_step=parse_optional_float(parts[12]) if len(parts) > 12 else None,
                    buck_duty=parse_optional_float(parts[13]) if len(parts) > 13 else None,
                    boost_duty=parse_optional_float(parts[14]) if len(parts) > 14 else None,
                    reference_power_w=(
                        parse_optional_float(parts[15]) if len(parts) > 15 else None
                    ),
                    sampled_power_w=(
                        parse_optional_float(parts[16]) if len(parts) > 16 else None
                    ),
                )

            # Compatibility with the older packet that had no sensor fields.
            if len(parts) >= 10:
                return TelemetryPacket(
                    unix_ms=received_unix_ms,
                    i_in_raw=int(parts[3]),
                    i_out_raw=int(parts[4]),
                    v_out_raw=int(parts[5]),
                    v_in_raw=int(parts[6]),
                    valid=parts[7] == "1",
                    temp0_c=None,
                    temp1_c=None,
                    irr_w_m2=None,
                    state=parts[8],
                    fault=parts[9],
                )
        except ValueError:
            return None

        return None

    def _format_telemetry_line(self, line: str, received_unix_ms: int) -> str:
        parts = line.split(",")
        if parts[0] == "ADC":
            parts = parts[3:]
        return f"{received_unix_ms},{','.join(parts)}"

    def _handle_packet(self, packet: TelemetryPacket) -> None:

        i_in_a = input_current_from_raw(packet.i_in_raw)
        i_out_a = output_current_from_raw(packet.i_out_raw)
        v_out_v = output_voltage_from_raw(packet.v_out_raw)
        v_in_v = input_voltage_from_raw(packet.v_in_raw)

        self.current_buffers["i_in_a"].append(i_in_a)
        self.current_buffers["i_out_a"].append(i_out_a)
        self.voltage_buffers["v_in_v"].append(v_in_v)
        self.voltage_buffers["v_out_v"].append(v_out_v)
        self.temperature_buffers["temp0_c"].append(packet.temp0_c)
        self.temperature_buffers["temp1_c"].append(packet.temp1_c)
        self.irradiance_buffers["irr_w_m2"].append(packet.irr_w_m2)

        # A leading blank reserves the sign column for positive values. Combined
        # with the fixed-width font above, crossing zero no longer moves digits
        # or changes the requested width of a summary cell.
        self.measurement_values["i_in_a"].set(f"{i_in_a: .2f}")
        self.measurement_values["i_out_a"].set(f"{i_out_a: .2f}")
        self.measurement_values["v_out_v"].set(f"{v_out_v: .2f}")
        self.measurement_values["v_in_v"].set(f"{v_in_v: .2f}")
        self.sensor_values["temp0_c"].set(format_optional(packet.temp0_c, "{: .2f}"))
        self.sensor_values["temp1_c"].set(format_optional(packet.temp1_c, "{: .2f}"))
        self.sensor_values["irr_w_m2"].set(format_optional(packet.irr_w_m2, "{: .0f}"))
        self.state_value.set(packet.state)
        self.fault_value.set(packet.fault)
        self.valid_value.set("yes" if packet.valid else "no")
        self.packet_status.set(f"Last packet: unix_ms={packet.unix_ms}")

    def _append_terminal(self, text: str) -> None:
        self.terminal.configure(state=tk.NORMAL)
        self.terminal.insert(tk.END, text + "\n")
        self.terminal_line_count += 1
        if self.terminal_line_count > TERMINAL_MAX_LINES:
            self.terminal.delete("1.0", "2.0")
            self.terminal_line_count -= 1
        self.terminal.see(tk.END)
        self.terminal.configure(state=tk.DISABLED)

    def _draw_plot(self) -> None:
        self._draw_series_plot(
            self.current_canvas,
            self.current_buffers,
            {"i_in_a": "#2563eb", "i_out_a": "#16a34a"},
            {"i_in_a": "I in", "i_out_a": "I out"},
            "Current (A)",
            "Waiting for current data",
            1.0,
        )
        self._draw_series_plot(
            self.voltage_canvas,
            self.voltage_buffers,
            {"v_in_v": "#9333ea", "v_out_v": "#dc2626"},
            {"v_in_v": "V in", "v_out_v": "V out"},
            "Voltage (V)",
            "Waiting for voltage data",
            5.0,
        )
        self._draw_series_plot(
            self.temperature_canvas,
            self.temperature_buffers,
            {"temp0_c": "#0f766e", "temp1_c": "#f97316"},
            {"temp0_c": "Temp0", "temp1_c": "Temp1"},
            "Temperature (C)",
            "Waiting for temperature data",
            5.0,
        )
        self._draw_series_plot(
            self.irradiance_canvas,
            self.irradiance_buffers,
            {"irr_w_m2": "#7c3aed"},
            {"irr_w_m2": "Irradiance"},
            "Irradiance (W/m2)",
            "Waiting for irradiance data",
            10.0,
        )
        self.after(100, self._draw_plot)

    def _draw_series_plot(
        self,
        canvas: tk.Canvas,
        series: dict[str, Deque[float | None]],
        colors: dict[str, str],
        labels: dict[str, str],
        title: str,
        empty_text: str,
        min_span: float,
    ) -> None:
        width = max(canvas.winfo_width(), 220)
        height = max(canvas.winfo_height(), 170)
        margin_left = 54
        margin_right = 16
        margin_top = 24
        margin_bottom = 30
        plot_width = width - margin_left - margin_right
        plot_height = height - margin_top - margin_bottom

        canvas.delete("all")
        canvas.create_rectangle(
            margin_left,
            margin_top,
            width - margin_right,
            height - margin_bottom,
            outline="#d1d5db",
            fill="#ffffff",
        )
        canvas.create_text(
            margin_left,
            10,
            text=title,
            anchor="w",
            fill="#374151",
            font=("Segoe UI", 9, "bold"),
        )

        all_values = [
            value
            for values in series.values()
            for value in values
            if value is not None
        ]
        if not all_values:
            canvas.create_text(
                width // 2,
                height // 2,
                text=empty_text,
                fill="#6b7280",
                font=("Segoe UI", 9),
            )
            return

        y_min = min(all_values)
        y_max = max(all_values)
        if y_min == y_max:
            padding = min_span / 2
            y_min -= padding
            y_max += padding
        else:
            padding = (y_max - y_min) * 0.1
            y_min -= padding
            y_max += padding
            if (y_max - y_min) < min_span:
                midpoint = (y_min + y_max) / 2
                y_min = midpoint - (min_span / 2)
                y_max = midpoint + (min_span / 2)

        for tick_index in range(5):
            value = y_min + (tick_index / 4) * (y_max - y_min)
            y = margin_top + plot_height - ((value - y_min) / (y_max - y_min)) * plot_height
            canvas.create_line(margin_left, y, width - margin_right, y, fill="#eef2f7")
            canvas.create_text(
                margin_left - 8,
                y,
                text=self._format_axis_value(value, y_max - y_min),
                anchor="e",
                fill="#6b7280",
                font=("Segoe UI", 8),
            )

        point_count = max((len(values) for values in series.values()), default=0)
        if point_count >= 2:
            for name, values in series.items():
                segment: list[float] = []
                for index, value in enumerate(values):
                    if value is None:
                        if len(segment) >= 4:
                            canvas.create_line(segment, fill=colors[name], width=2)
                        segment = []
                        continue
                    x = margin_left + (index / max(1, point_count - 1)) * plot_width
                    y = margin_top + plot_height - ((value - y_min) / (y_max - y_min)) * plot_height
                    segment.extend((x, y))
                if len(segment) >= 4:
                    canvas.create_line(segment, fill=colors[name], width=2)

        legend_x = margin_left + 8
        legend_y = margin_top + 8
        for index, (name, color) in enumerate(colors.items()):
            x = legend_x + index * 92
            canvas.create_rectangle(x, legend_y, x + 12, legend_y + 12, fill=color, width=0)
            canvas.create_text(
                x + 18,
                legend_y + 6,
                text=labels[name],
                anchor="w",
                fill="#374151",
                font=("Segoe UI", 8),
            )

        canvas.create_text(
            width // 2,
            height - 10,
            text=f"Last {MAX_POINTS} packets, autoscaled",
            fill="#6b7280",
            font=("Segoe UI", 8),
        )

    def _format_axis_value(self, value: float, span: float) -> str:
        if span >= 100:
            return f"{value:.0f}"
        if span >= 10:
            return f"{value:.1f}"
        return f"{value:.2f}"

    def destroy(self) -> None:
        self.disconnect()
        super().destroy()


if __name__ == "__main__":
    app = MpptSerialGui()
    app.mainloop()
