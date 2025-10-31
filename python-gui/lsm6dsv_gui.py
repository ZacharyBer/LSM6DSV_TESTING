#!/usr/bin/env python3
"""
LSM6DSV Sensor Interface GUI
Main application for visualizing and configuring LSM6DSV sensor data
"""

import sys
import tkinter as tk
from tkinter import ttk, scrolledtext
import serial
import serial.tools.list_ports
import threading
import queue
import time
from collections import deque
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np


class LSM6DSV_GUI:
    """Main GUI application for LSM6DSV sensor interface"""

    def __init__(self, root):
        self.root = root
        self.root.title("LSM6DSV Sensor Interface")
        self.root.geometry("1400x900")

        # Serial communication
        self.serial_port = None
        self.serial_thread = None
        self.running = False
        self.data_queue = queue.Queue()

        # Data buffers (store last 1000 samples)
        self.time_data = deque(maxlen=1000)
        self.accel_x = deque(maxlen=1000)
        self.accel_y = deque(maxlen=1000)
        self.accel_z = deque(maxlen=1000)
        self.gyro_x = deque(maxlen=1000)
        self.gyro_y = deque(maxlen=1000)
        self.gyro_z = deque(maxlen=1000)
        self.quat_w = deque(maxlen=1000)
        self.quat_x = deque(maxlen=1000)
        self.quat_y = deque(maxlen=1000)
        self.quat_z = deque(maxlen=1000)

        # Statistics
        self.sample_count = 0
        self.parse_error_count = 0
        self.empty_field_count = 0
        self.start_time = time.time()
        self.last_update = time.time()

        # Interrupt event tracking
        self.interrupt_events = deque(maxlen=1000)  # Store recent events with timestamps
        self.event_counters = {
            'WAKE_UP': 0,
            'SINGLE_TAP': 0,
            'DOUBLE_TAP': 0,
            'FREE_FALL': 0,
            '6D_ORIENT': 0,
            'TILT': 0,
            'STEP_DET': 0,
            'FIFO_FULL': 0,
            'FIFO_WM': 0,
            'DATA_READY': 0
        }

        # Create GUI
        self.create_widgets()

        # Start update loop
        self.update_data()

    def create_widgets(self):
        """Create all GUI widgets"""

        # ===== Top Control Panel =====
        control_frame = ttk.Frame(self.root, padding="5")
        control_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E))

        # Serial Port Selection
        ttk.Label(control_frame, text="Serial Port:").grid(row=0, column=0, padx=5)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(control_frame, textvariable=self.port_var, width=20)
        self.port_combo.grid(row=0, column=1, padx=5)
        self.refresh_ports()

        ttk.Button(control_frame, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=5)

        # Baud Rate
        ttk.Label(control_frame, text="Baud Rate:").grid(row=0, column=3, padx=5)
        self.baud_var = tk.StringVar(value="921600")
        baud_combo = ttk.Combobox(control_frame, textvariable=self.baud_var,
                                   values=["115200", "921600", "1000000"], width=10)
        baud_combo.grid(row=0, column=4, padx=5)

        # Connect/Disconnect Button
        self.connect_button = ttk.Button(control_frame, text="Connect", command=self.toggle_connection)
        self.connect_button.grid(row=0, column=5, padx=10)

        # Status
        self.status_var = tk.StringVar(value="Disconnected")
        self.status_label = ttk.Label(control_frame, textvariable=self.status_var, foreground="red")
        self.status_label.grid(row=0, column=6, padx=10)

        # Statistics
        self.stats_var = tk.StringVar(value="Samples: 0 | Rate: 0 Hz")
        ttk.Label(control_frame, textvariable=self.stats_var).grid(row=0, column=7, padx=10)

        # ===== Create Notebook for Tabs =====
        notebook = ttk.Notebook(self.root)
        notebook.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), padx=5, pady=5)

        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=1)

        # ===== Tab 1: Real-Time Plots =====
        plot_frame = ttk.Frame(notebook)
        notebook.add(plot_frame, text="Data Visualization")

        # Create matplotlib figures
        self.fig = Figure(figsize=(14, 8))

        # Accelerometer plot (top)
        self.ax_accel = self.fig.add_subplot(3, 1, 1)
        self.ax_accel.set_title("Accelerometer (mg)", fontsize=10)
        self.ax_accel.set_xlabel("Time (s)")
        self.ax_accel.set_ylabel("Acceleration (mg)")
        self.ax_accel.grid(True, alpha=0.3)
        self.line_accel_x, = self.ax_accel.plot([], [], 'r-', label='X', linewidth=1)
        self.line_accel_y, = self.ax_accel.plot([], [], 'g-', label='Y', linewidth=1)
        self.line_accel_z, = self.ax_accel.plot([], [], 'b-', label='Z', linewidth=1)
        self.ax_accel.legend(loc='upper right', fontsize=8)

        # Gyroscope plot (middle)
        self.ax_gyro = self.fig.add_subplot(3, 1, 2)
        self.ax_gyro.set_title("Gyroscope (mdps)", fontsize=10)
        self.ax_gyro.set_xlabel("Time (s)")
        self.ax_gyro.set_ylabel("Angular Rate (mdps)")
        self.ax_gyro.grid(True, alpha=0.3)
        self.line_gyro_x, = self.ax_gyro.plot([], [], 'r-', label='X', linewidth=1)
        self.line_gyro_y, = self.ax_gyro.plot([], [], 'g-', label='Y', linewidth=1)
        self.line_gyro_z, = self.ax_gyro.plot([], [], 'b-', label='Z', linewidth=1)
        self.ax_gyro.legend(loc='upper right', fontsize=8)

        # Quaternion plot (bottom)
        self.ax_quat = self.fig.add_subplot(3, 1, 3)
        self.ax_quat.set_title("Sensor Fusion Quaternion (SFLP)", fontsize=10)
        self.ax_quat.set_xlabel("Time (s)")
        self.ax_quat.set_ylabel("Quaternion")
        self.ax_quat.grid(True, alpha=0.3)
        self.line_quat_w, = self.ax_quat.plot([], [], 'k-', label='W', linewidth=1)
        self.line_quat_x, = self.ax_quat.plot([], [], 'r-', label='X', linewidth=1)
        self.line_quat_y, = self.ax_quat.plot([], [], 'g-', label='Y', linewidth=1)
        self.line_quat_z, = self.ax_quat.plot([], [], 'b-', label='Z', linewidth=1)
        self.ax_quat.legend(loc='upper right', fontsize=8)

        self.fig.tight_layout()

        # Embed matplotlib in tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # ===== Tab 2: Configuration =====
        config_frame = ttk.Frame(notebook, padding="10")
        notebook.add(config_frame, text="Configuration")

        # Sensor Configuration
        sensor_group = ttk.LabelFrame(config_frame, text="Sensor Configuration", padding="10")
        sensor_group.grid(row=0, column=0, padx=10, pady=10, sticky=(tk.W, tk.E, tk.N))

        # I2C Address
        ttk.Label(sensor_group, text="I2C Address:").grid(row=0, column=0, sticky=tk.W, pady=5)
        self.i2c_addr_var = tk.StringVar(value="0x6B")
        i2c_combo = ttk.Combobox(sensor_group, textvariable=self.i2c_addr_var,
                                  values=["0x6A", "0x6B"], width=10)
        i2c_combo.grid(row=0, column=1, sticky=tk.W, pady=5, padx=5)

        # Accelerometer ODR
        ttk.Label(sensor_group, text="Accel ODR:").grid(row=1, column=0, sticky=tk.W, pady=5)
        self.acc_odr_var = tk.StringVar(value="120 Hz")
        acc_odr_combo = ttk.Combobox(sensor_group, textvariable=self.acc_odr_var,
                                      values=["7.5 Hz", "15 Hz", "30 Hz", "60 Hz", "120 Hz",
                                              "240 Hz", "480 Hz", "960 Hz"], width=10)
        acc_odr_combo.grid(row=1, column=1, sticky=tk.W, pady=5, padx=5)

        # Accelerometer Full Scale
        ttk.Label(sensor_group, text="Accel FS:").grid(row=2, column=0, sticky=tk.W, pady=5)
        self.acc_fs_var = tk.StringVar(value="±4g")
        acc_fs_combo = ttk.Combobox(sensor_group, textvariable=self.acc_fs_var,
                                     values=["±2g", "±4g", "±8g", "±16g"], width=10)
        acc_fs_combo.grid(row=2, column=1, sticky=tk.W, pady=5, padx=5)

        # Gyroscope ODR
        ttk.Label(sensor_group, text="Gyro ODR:").grid(row=3, column=0, sticky=tk.W, pady=5)
        self.gyro_odr_var = tk.StringVar(value="120 Hz")
        gyro_odr_combo = ttk.Combobox(sensor_group, textvariable=self.gyro_odr_var,
                                       values=["7.5 Hz", "15 Hz", "30 Hz", "60 Hz", "120 Hz",
                                               "240 Hz", "480 Hz", "960 Hz"], width=10)
        gyro_odr_combo.grid(row=3, column=1, sticky=tk.W, pady=5, padx=5)

        # Gyroscope Full Scale
        ttk.Label(sensor_group, text="Gyro FS:").grid(row=4, column=0, sticky=tk.W, pady=5)
        self.gyro_fs_var = tk.StringVar(value="±2000 dps")
        gyro_fs_combo = ttk.Combobox(sensor_group, textvariable=self.gyro_fs_var,
                                      values=["±125 dps", "±250 dps", "±500 dps",
                                              "±1000 dps", "±2000 dps", "±4000 dps"], width=10)
        gyro_fs_combo.grid(row=4, column=1, sticky=tk.W, pady=5, padx=5)

        # Apply button
        ttk.Button(sensor_group, text="Apply Configuration",
                   command=self.apply_config).grid(row=5, column=0, columnspan=2, pady=10)

        # SFLP Configuration
        sflp_group = ttk.LabelFrame(config_frame, text="Sensor Fusion (SFLP)", padding="10")
        sflp_group.grid(row=1, column=0, padx=10, pady=10, sticky=(tk.W, tk.E, tk.N))

        self.sflp_enabled_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(sflp_group, text="Enable SFLP",
                        variable=self.sflp_enabled_var,
                        command=self.toggle_sflp).grid(row=0, column=0, sticky=tk.W, pady=5)

        ttk.Label(sflp_group, text="SFLP ODR:").grid(row=1, column=0, sticky=tk.W, pady=5)
        self.sflp_odr_var = tk.StringVar(value="15 Hz")
        sflp_odr_combo = ttk.Combobox(sflp_group, textvariable=self.sflp_odr_var,
                                       values=["15 Hz", "30 Hz", "60 Hz", "120 Hz"], width=10)
        sflp_odr_combo.grid(row=1, column=1, sticky=tk.W, pady=5, padx=5)

        # ===== Tab 3: Interrupt Events =====
        interrupt_frame = ttk.Frame(notebook, padding="10")
        notebook.add(interrupt_frame, text="Interrupt Events")

        # Event Controls
        control_group = ttk.LabelFrame(interrupt_frame, text="Embedded Functions Control", padding="10")
        control_group.grid(row=0, column=0, padx=10, pady=10, sticky=(tk.W, tk.E, tk.N))

        # Wake-up detection
        wake_frame = ttk.Frame(control_group)
        wake_frame.grid(row=0, column=0, columnspan=3, sticky=tk.W, pady=5)
        self.wake_enabled_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(wake_frame, text="Wake-Up Detection", variable=self.wake_enabled_var,
                       command=lambda: self.send_command(f"{'ENABLE' if self.wake_enabled_var.get() else 'DISABLE'}:WAKE_UP")).pack(side=tk.LEFT)
        ttk.Label(wake_frame, text="Threshold (mg):").pack(side=tk.LEFT, padx=(20,5))
        self.wake_thresh_var = tk.StringVar(value="100")
        wake_thresh_entry = ttk.Entry(wake_frame, textvariable=self.wake_thresh_var, width=8)
        wake_thresh_entry.pack(side=tk.LEFT)
        ttk.Button(wake_frame, text="Set", command=lambda: self.send_command(f"SET:WAKE_THRESHOLD:{self.wake_thresh_var.get()}")).pack(side=tk.LEFT, padx=5)

        # Tap detection
        tap_frame = ttk.Frame(control_group)
        tap_frame.grid(row=1, column=0, columnspan=3, sticky=tk.W, pady=5)
        self.tap_enabled_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(tap_frame, text="Tap Detection", variable=self.tap_enabled_var,
                       command=lambda: self.send_command(f"{'ENABLE' if self.tap_enabled_var.get() else 'DISABLE'}:TAP")).pack(side=tk.LEFT)
        ttk.Label(tap_frame, text="Threshold:").pack(side=tk.LEFT, padx=(20,5))
        self.tap_thresh_var = tk.StringVar(value="15")
        tap_thresh_entry = ttk.Entry(tap_frame, textvariable=self.tap_thresh_var, width=8)
        tap_thresh_entry.pack(side=tk.LEFT)
        ttk.Button(tap_frame, text="Set X/Y/Z", command=lambda: self.send_command(f"SET:TAP_THRESHOLD:{self.tap_thresh_var.get()}")).pack(side=tk.LEFT, padx=5)

        # Free fall detection
        ff_frame = ttk.Frame(control_group)
        ff_frame.grid(row=2, column=0, columnspan=3, sticky=tk.W, pady=5)
        self.ff_enabled_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(ff_frame, text="Free Fall Detection", variable=self.ff_enabled_var,
                       command=lambda: self.send_command(f"{'ENABLE' if self.ff_enabled_var.get() else 'DISABLE'}:FREE_FALL")).pack(side=tk.LEFT)
        ttk.Label(ff_frame, text="Threshold (mg):").pack(side=tk.LEFT, padx=(20,5))
        self.ff_thresh_var = tk.StringVar(value="156")
        ff_thresh_entry = ttk.Entry(ff_frame, textvariable=self.ff_thresh_var, width=8)
        ff_thresh_entry.pack(side=tk.LEFT)
        ttk.Button(ff_frame, text="Set", command=lambda: self.send_command(f"SET:FF_THRESHOLD:{self.ff_thresh_var.get()}")).pack(side=tk.LEFT, padx=5)

        # 6D Orientation
        sixd_frame = ttk.Frame(control_group)
        sixd_frame.grid(row=3, column=0, columnspan=3, sticky=tk.W, pady=5)
        self.sixd_enabled_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(sixd_frame, text="6D Orientation", variable=self.sixd_enabled_var,
                       command=lambda: self.send_command(f"{'ENABLE' if self.sixd_enabled_var.get() else 'DISABLE'}:6D")).pack(side=tk.LEFT)

        # Tilt Detection
        tilt_frame = ttk.Frame(control_group)
        tilt_frame.grid(row=4, column=0, columnspan=3, sticky=tk.W, pady=5)
        self.tilt_enabled_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(tilt_frame, text="Tilt Detection", variable=self.tilt_enabled_var,
                       command=lambda: self.send_command(f"{'ENABLE' if self.tilt_enabled_var.get() else 'DISABLE'}:TILT")).pack(side=tk.LEFT)

        # Event Log
        log_group = ttk.LabelFrame(interrupt_frame, text="Event Log", padding="10")
        log_group.grid(row=1, column=0, padx=10, pady=10, sticky=(tk.W, tk.E, tk.N, tk.S))

        self.interrupt_log = scrolledtext.ScrolledText(log_group, height=15, width=60,
                                                        wrap=tk.WORD, font=("Courier", 9))
        self.interrupt_log.pack(fill=tk.BOTH, expand=True)

        ttk.Button(log_group, text="Clear Log",
                  command=lambda: self.interrupt_log.delete(1.0, tk.END)).pack(pady=5)

        # Event Counters
        counter_group = ttk.LabelFrame(interrupt_frame, text="Event Counters", padding="10")
        counter_group.grid(row=0, column=1, rowspan=2, padx=10, pady=10, sticky=(tk.W, tk.E, tk.N))

        self.event_count_labels = {}
        row = 0
        for event in ['WAKE_UP', 'SINGLE_TAP', 'DOUBLE_TAP', 'FREE_FALL', '6D_ORIENT', 'TILT', 'STEP_DET']:
            ttk.Label(counter_group, text=f"{event}:").grid(row=row, column=0, sticky=tk.W, pady=2)
            label = ttk.Label(counter_group, text="0", width=8)
            label.grid(row=row, column=1, sticky=tk.E, pady=2, padx=5)
            self.event_count_labels[event] = label
            row += 1

        ttk.Button(counter_group, text="Reset Counters",
                  command=self.reset_event_counters).grid(row=row, column=0, columnspan=2, pady=10)

        # ===== Tab 4: Console/Log =====
        console_frame = ttk.Frame(notebook, padding="5")
        notebook.add(console_frame, text="Console")

        # Console text area
        self.console_text = scrolledtext.ScrolledText(console_frame, height=40, width=120,
                                                       wrap=tk.WORD, font=("Courier", 9))
        self.console_text.pack(fill=tk.BOTH, expand=True)

        # Clear console button
        ttk.Button(console_frame, text="Clear Console",
                   command=lambda: self.console_text.delete(1.0, tk.END)).pack(pady=5)

    def refresh_ports(self):
        """Refresh available serial ports"""
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports and not self.port_var.get():
            self.port_combo.current(0)

    def toggle_connection(self):
        """Connect or disconnect from serial port"""
        if not self.running:
            self.connect()
        else:
            self.disconnect()

    def connect(self):
        """Connect to serial port"""
        port = self.port_var.get()
        baud = int(self.baud_var.get())

        if not port:
            self.log_message("ERROR: No serial port selected\n")
            return

        try:
            self.serial_port = serial.Serial(port, baud, timeout=0.1)
            self.running = True

            # Start serial thread
            self.serial_thread = threading.Thread(target=self.serial_worker, daemon=True)
            self.serial_thread.start()

            # Update UI
            self.connect_button.config(text="Disconnect")
            self.status_var.set(f"Connected: {port} @ {baud} baud")
            self.status_label.config(foreground="green")

            self.log_message(f"Connected to {port} @ {baud} baud\n")

        except Exception as e:
            self.log_message(f"ERROR: Failed to connect: {str(e)}\n")

    def disconnect(self):
        """Disconnect from serial port"""
        self.running = False

        if self.serial_thread:
            self.serial_thread.join(timeout=2)

        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()

        # Update UI
        self.connect_button.config(text="Connect")
        self.status_var.set("Disconnected")
        self.status_label.config(foreground="red")

        self.log_message("Disconnected\n")

    def send_command(self, cmd):
        """Send command to firmware via UART"""
        if self.serial_port and self.serial_port.is_open:
            try:
                cmd_bytes = (cmd + '\r\n').encode('utf-8')
                self.serial_port.write(cmd_bytes)
                self.log_message(f"TX: {cmd}\n")
                return True
            except Exception as e:
                self.log_message(f"ERROR sending command: {e}\n")
                return False
        else:
            self.log_message("ERROR: Not connected to serial port\n")
            return False

    def serial_worker(self):
        """Serial port reading thread"""
        buffer = ""

        while self.running:
            try:
                if self.serial_port.in_waiting > 0:
                    data = self.serial_port.read(self.serial_port.in_waiting).decode('utf-8', errors='ignore')
                    buffer += data

                    # Process complete lines
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        if line:
                            self.data_queue.put(line)
                else:
                    time.sleep(0.001)  # Small delay to prevent CPU spinning

            except Exception as e:
                if self.running:
                    self.log_message(f"Serial Error: {str(e)}\n")
                break

    def parse_data(self, line):
        """Parse incoming CSV data and interrupt messages"""
        try:
            # Check for interrupt events (INT:EVENT_TYPE format)
            if line.startswith("INT:"):
                event_type = line[4:].strip()  # Remove "INT:" prefix

                # Update event counter
                if event_type in self.event_counters:
                    self.event_counters[event_type] += 1

                # Store event with timestamp
                event_timestamp = time.time() - self.start_time
                self.interrupt_events.append((event_timestamp, event_type))

                # Update interrupt log if it exists
                if hasattr(self, 'interrupt_log'):
                    self.interrupt_log.insert('1.0',
                        f"[{event_timestamp:.2f}s] {event_type}\n")
                    # Update event counters display
                    if hasattr(self, 'event_count_labels'):
                        for event, label in self.event_count_labels.items():
                            label.config(text=str(self.event_counters.get(event, 0)))

                return True

            parts = line.split(',')

            if parts[0] == "LSM6DSV" and len(parts) >= 8:
                # LSM6DSV,timestamp,ax,ay,az,gx,gy,gz

                # Validate that all required fields are non-empty
                for i in range(1, 8):
                    if not parts[i] or parts[i].strip() == '':
                        self.empty_field_count += 1
                        self.log_message(f"PARSE ERROR: Empty field in CSV at position {i}: '{line}'\n")
                        self.log_message(f"  Fields: {parts}\n")
                        return False

                try:
                    timestamp = float(parts[1]) / 1000000.0  # Convert to seconds
                    ax = float(parts[2])
                    ay = float(parts[3])
                    az = float(parts[4])
                    gx = float(parts[5])
                    gy = float(parts[6])
                    gz = float(parts[7])
                except ValueError as ve:
                    self.parse_error_count += 1
                    self.log_message(f"PARSE ERROR: Invalid float value: {ve}\n")
                    self.log_message(f"  Line: '{line}'\n")
                    self.log_message(f"  Fields: {parts}\n")
                    return False

                # Store data
                self.time_data.append(timestamp)
                self.accel_x.append(ax)
                self.accel_y.append(ay)
                self.accel_z.append(az)
                self.gyro_x.append(gx)
                self.gyro_y.append(gy)
                self.gyro_z.append(gz)

                self.sample_count += 1
                return True

            elif parts[0] == "LSM6DSV_SFLP" and len(parts) >= 6:
                # LSM6DSV_SFLP,timestamp,qw,qx,qy,qz

                # Validate that all required fields are non-empty
                for i in range(1, 6):
                    if not parts[i] or parts[i].strip() == '':
                        self.empty_field_count += 1
                        self.log_message(f"PARSE ERROR: Empty SFLP field at position {i}: '{line}'\n")
                        return False

                try:
                    timestamp = float(parts[1]) / 1000000.0
                    qw = float(parts[2])
                    qx = float(parts[3])
                    qy = float(parts[4])
                    qz = float(parts[5])
                except ValueError as ve:
                    self.parse_error_count += 1
                    self.log_message(f"PARSE ERROR: Invalid SFLP float value: {ve}\n")
                    self.log_message(f"  Line: '{line}'\n")
                    return False

                # Store quaternion data
                self.quat_w.append(qw)
                self.quat_x.append(qx)
                self.quat_y.append(qy)
                self.quat_z.append(qz)
                return True

        except Exception as e:
            self.parse_error_count += 1
            self.log_message(f"PARSE ERROR: Unexpected exception: {e}\n")
            self.log_message(f"  Line: '{line}'\n")
            return False

        return False

    def update_data(self):
        """Update GUI with new data (called periodically)"""
        # Process queued data
        data_received = False
        while not self.data_queue.empty():
            try:
                line = self.data_queue.get_nowait()
                if not self.parse_data(line):
                    # Not sensor data, log to console
                    self.log_message(line + '\n')
                else:
                    data_received = True
            except queue.Empty:
                break

        # Update plots if data received
        if data_received and len(self.time_data) > 0:
            self.update_plots()

        # Update statistics
        current_time = time.time()
        if current_time - self.last_update >= 1.0:  # Update every second
            elapsed = current_time - self.start_time
            rate = self.sample_count / elapsed if elapsed > 0 else 0
            error_info = ""
            if self.parse_error_count > 0 or self.empty_field_count > 0:
                error_info = f" | Errors: {self.parse_error_count} | Empty: {self.empty_field_count}"
            self.stats_var.set(f"Samples: {self.sample_count} | Rate: {rate:.1f} Hz{error_info}")
            self.last_update = current_time

        # Schedule next update
        self.root.after(50, self.update_data)  # 20 Hz update rate

    def update_plots(self):
        """Update all plots with current data"""
        if len(self.time_data) == 0:
            return

        # Convert to numpy arrays for plotting
        time_array = np.array(self.time_data)
        time_array = time_array - time_array[0]  # Normalize to start at 0

        # Update accelerometer plot
        self.line_accel_x.set_data(time_array, self.accel_x)
        self.line_accel_y.set_data(time_array, self.accel_y)
        self.line_accel_z.set_data(time_array, self.accel_z)
        self.ax_accel.relim()
        self.ax_accel.autoscale_view()

        # Update gyroscope plot
        self.line_gyro_x.set_data(time_array, self.gyro_x)
        self.line_gyro_y.set_data(time_array, self.gyro_y)
        self.line_gyro_z.set_data(time_array, self.gyro_z)
        self.ax_gyro.relim()
        self.ax_gyro.autoscale_view()

        # Update quaternion plot if data available
        if len(self.quat_w) > 0:
            quat_time = time_array[-len(self.quat_w):]  # Match quaternion data length
            self.line_quat_w.set_data(quat_time, self.quat_w)
            self.line_quat_x.set_data(quat_time, self.quat_x)
            self.line_quat_y.set_data(quat_time, self.quat_y)
            self.line_quat_z.set_data(quat_time, self.quat_z)
            self.ax_quat.relim()
            self.ax_quat.autoscale_view()

        # Redraw canvas
        self.canvas.draw_idle()

    def log_message(self, message):
        """Log message to console"""
        self.console_text.insert(tk.END, message)
        self.console_text.see(tk.END)

    def apply_config(self):
        """Apply sensor configuration via commands"""
        self.log_message("Applying configuration...\n")

        # Parse and send accelerometer ODR
        acc_odr = self.acc_odr_var.get().split()[0]  # "120 Hz" -> "120"
        self.send_command(f"SET:ACC_ODR:{acc_odr}")

        # Parse and send accelerometer full-scale
        acc_fs = self.acc_fs_var.get().replace('±', '').replace('g', 'G')  # "±4g" -> "4G"
        self.send_command(f"SET:ACC_FS:{acc_fs}")

        # Parse and send gyro ODR
        gyro_odr = self.gyro_odr_var.get().split()[0]  # "120 Hz" -> "120"
        self.send_command(f"SET:GYRO_ODR:{gyro_odr}")

        # Parse and send gyro full-scale
        gyro_fs = self.gyro_fs_var.get().replace('±', '').replace(' dps', 'DPS')  # "±2000 dps" -> "2000DPS"
        self.send_command(f"SET:GYRO_FS:{gyro_fs}")

        self.log_message("Configuration commands sent\n")

    def toggle_sflp(self):
        """Toggle SFLP enable"""
        enabled = self.sflp_enabled_var.get()
        if enabled:
            self.send_command(f"ENABLE:SFLP")
            sflp_odr = self.sflp_odr_var.get().split()[0]  # "15 Hz" -> "15"
            self.send_command(f"SET:SFLP_ODR:{sflp_odr}")
        else:
            self.send_command(f"DISABLE:SFLP")
        self.log_message(f"SFLP {'Enabled' if enabled else 'Disabled'}\n")

    def reset_event_counters(self):
        """Reset all interrupt event counters"""
        for event in self.event_counters.keys():
            self.event_counters[event] = 0

        # Update display
        for event, label in self.event_count_labels.items():
            label.config(text="0")

        self.log_message("Event counters reset\n")

    def on_closing(self):
        """Handle window closing"""
        self.disconnect()
        self.root.destroy()


def main():
    """Main entry point"""
    root = tk.Tk()
    app = LSM6DSV_GUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()
