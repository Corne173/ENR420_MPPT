"""Minimal serial GUI for the ENR MPPT controller.

Expected telemetry packet:
ADC,ms,seq,i_in_raw,i_out_raw,v_out_raw,v_in_raw,valid,state,fault
"""

from __future__ import annotations

import queue
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque

import tkinter as tk
from tkinter import messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover - handled at runtime in the GUI
    serial = None
    list_ports = None


BAUD_DEFAULT = 230400
MAX_POINTS = 600
TERMINAL_MAX_LINES = 500
ADC_MAX_COUNTS = 4095


@dataclass(frozen=True)
class AdcPacket:
    timestamp_ms: int
    sequence: int
    i_in_raw: int
    i_out_raw: int
    v_out_raw: int
    v_in_raw: int
    valid: bool
    state: str
    fault: str


class SerialReader(threading.Thread):
    def __init__(self, port: str, baudrate: int, output_queue: queue.Queue[str]):
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
                    self._output_queue.put(text)

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
        self.serial_queue: queue.Queue[str] = queue.Queue()
        self.terminal_line_count = 0
        self.last_sequence: int | None = None
        self.dropped_packets = 0

        self.time_points: Deque[int] = deque(maxlen=MAX_POINTS)
        self.adc_buffers: dict[str, Deque[int]] = {
            "i_in_raw": deque(maxlen=MAX_POINTS),
            "i_out_raw": deque(maxlen=MAX_POINTS),
            "v_out_raw": deque(maxlen=MAX_POINTS),
            "v_in_raw": deque(maxlen=MAX_POINTS),
        }

        self.connection_status = tk.StringVar(value="Disconnected")
        self.packet_status = tk.StringVar(value="No ADC packets received")
        self.state_value = tk.StringVar(value="-")
        self.fault_value = tk.StringVar(value="-")
        self.valid_value = tk.StringVar(value="-")
        self.drop_value = tk.StringVar(value="0")
        self.raw_values = {
            "i_in_raw": tk.StringVar(value="-"),
            "i_out_raw": tk.StringVar(value="-"),
            "v_out_raw": tk.StringVar(value="-"),
            "v_in_raw": tk.StringVar(value="-"),
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
        for column in range(8):
            summary.columnconfigure(column, weight=1)

        self._add_value(summary, "I in raw", self.raw_values["i_in_raw"], 0)
        self._add_value(summary, "I out raw", self.raw_values["i_out_raw"], 1)
        self._add_value(summary, "V out raw", self.raw_values["v_out_raw"], 2)
        self._add_value(summary, "V in raw", self.raw_values["v_in_raw"], 3)
        self._add_value(summary, "State", self.state_value, 4)
        self._add_value(summary, "Fault", self.fault_value, 5)
        self._add_value(summary, "Valid", self.valid_value, 6)
        self._add_value(summary, "Dropped", self.drop_value, 7)

        plot_frame = ttk.Frame(left)
        plot_frame.grid(row=1, column=0, sticky="nsew")
        plot_frame.rowconfigure(0, weight=1)
        plot_frame.columnconfigure(0, weight=1)
        self.plot_canvas = tk.Canvas(
            plot_frame,
            background="#ffffff",
            highlightthickness=1,
            highlightbackground="#d1d5db",
        )
        self.plot_canvas.grid(row=0, column=0, sticky="nsew")

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

        ttk.Label(right, text="Serial terminal", font=("Segoe UI", 11, "bold")).grid(
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

    def _add_value(
        self,
        parent: ttk.Frame,
        label: str,
        variable: tk.StringVar,
        column: int,
    ) -> None:
        frame = ttk.Frame(parent)
        frame.grid(row=0, column=column, sticky="ew", padx=3)
        ttk.Label(frame, text=label, font=("Segoe UI", 8)).pack(anchor="w")
        ttk.Label(
            frame,
            textvariable=variable,
            style="Value.TLabel",
            anchor="center",
            font=("Segoe UI", 11, "bold"),
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
        if self.reader is not None:
            self.reader.stop()
            self.reader = None
        self.connect_button.configure(text="Connect")
        self.connection_status.set("Disconnected")

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
                line = self.serial_queue.get_nowait()
            except queue.Empty:
                break

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
            packet = self._parse_adc_packet(line)
            if packet:
                self._handle_packet(packet)

        self.after(50, self._process_serial_queue)

    def _parse_adc_packet(self, line: str) -> AdcPacket | None:
        parts = line.split(",")
        if len(parts) < 10 or parts[0] != "ADC":
            return None

        try:
            return AdcPacket(
                timestamp_ms=int(parts[1]),
                sequence=int(parts[2]),
                i_in_raw=int(parts[3]),
                i_out_raw=int(parts[4]),
                v_out_raw=int(parts[5]),
                v_in_raw=int(parts[6]),
                valid=parts[7] == "1",
                state=parts[8],
                fault=parts[9],
            )
        except ValueError:
            return None

    def _handle_packet(self, packet: AdcPacket) -> None:
        if self.last_sequence is not None and packet.sequence != self.last_sequence + 1:
            self.dropped_packets += max(0, packet.sequence - self.last_sequence - 1)
        self.last_sequence = packet.sequence

        self.time_points.append(packet.timestamp_ms)
        self.adc_buffers["i_in_raw"].append(packet.i_in_raw)
        self.adc_buffers["i_out_raw"].append(packet.i_out_raw)
        self.adc_buffers["v_out_raw"].append(packet.v_out_raw)
        self.adc_buffers["v_in_raw"].append(packet.v_in_raw)

        self.raw_values["i_in_raw"].set(str(packet.i_in_raw))
        self.raw_values["i_out_raw"].set(str(packet.i_out_raw))
        self.raw_values["v_out_raw"].set(str(packet.v_out_raw))
        self.raw_values["v_in_raw"].set(str(packet.v_in_raw))
        self.state_value.set(packet.state)
        self.fault_value.set(packet.fault)
        self.valid_value.set("yes" if packet.valid else "no")
        self.drop_value.set(str(self.dropped_packets))
        self.packet_status.set(
            f"Last packet: t={packet.timestamp_ms} ms, seq={packet.sequence}"
        )

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
        canvas = self.plot_canvas
        width = max(canvas.winfo_width(), 200)
        height = max(canvas.winfo_height(), 200)
        margin_left = 52
        margin_right = 16
        margin_top = 22
        margin_bottom = 34
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

        for tick in range(0, ADC_MAX_COUNTS + 1, 1024):
            y = margin_top + plot_height - (tick / ADC_MAX_COUNTS) * plot_height
            canvas.create_line(margin_left, y, width - margin_right, y, fill="#eef2f7")
            canvas.create_text(
                margin_left - 8,
                y,
                text=str(tick),
                anchor="e",
                fill="#6b7280",
                font=("Segoe UI", 8),
            )

        colors = {
            "i_in_raw": "#2563eb",
            "i_out_raw": "#16a34a",
            "v_out_raw": "#dc2626",
            "v_in_raw": "#9333ea",
        }

        point_count = len(self.time_points)
        if point_count >= 2:
            for name, values in self.adc_buffers.items():
                points: list[float] = []
                for index, value in enumerate(values):
                    x = margin_left + (index / max(1, point_count - 1)) * plot_width
                    y = margin_top + plot_height - (value / ADC_MAX_COUNTS) * plot_height
                    points.extend((x, y))
                if len(points) >= 4:
                    canvas.create_line(points, fill=colors[name], width=2)

        legend_x = margin_left + 8
        legend_y = margin_top + 8
        for index, (name, color) in enumerate(colors.items()):
            x = legend_x + index * 112
            canvas.create_rectangle(x, legend_y, x + 12, legend_y + 12, fill=color, width=0)
            canvas.create_text(
                x + 18,
                legend_y + 6,
                text=name,
                anchor="w",
                fill="#374151",
                font=("Segoe UI", 8),
            )

        canvas.create_text(
            width // 2,
            height - 12,
            text=f"Last {MAX_POINTS} ADC packets",
            fill="#6b7280",
            font=("Segoe UI", 8),
        )

        self.after(100, self._draw_plot)

    def destroy(self) -> None:
        self.disconnect()
        super().destroy()


if __name__ == "__main__":
    app = MpptSerialGui()
    app.mainloop()
