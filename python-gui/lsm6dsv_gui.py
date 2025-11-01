#!/usr/bin/env python3
"""
LSM6DSV Sensor Interface GUI
Main application for visualizing and configuring LSM6DSV sensor data
"""

import sys
import tkinter as tk
from tkinter import ttk, scrolledtext, filedialog, messagebox
import serial
import serial.tools.list_ports
import threading
import queue
import time
from collections import deque
from datetime import datetime
import os
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
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

        # Data buffers (unlimited storage for continuous plotting)
        self.time_data = deque()
        self.accel_x = deque()
        self.accel_y = deque()
        self.accel_z = deque()
        self.gyro_x = deque()
        self.gyro_y = deque()
        self.gyro_z = deque()

        # Quaternion data with separate timestamp tracking
        self.quat_time_data = deque()
        self.quat_w = deque()
        self.quat_x = deque()
        self.quat_y = deque()
        self.quat_z = deque()

        # Euler angles (computed from quaternions)
        self.euler_roll = deque()
        self.euler_pitch = deque()
        self.euler_yaw = deque()

        # Data collection control
        self.paused = False

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
            'SIG_MOT': 0,
            'FIFO_FULL': 0,
            'FIFO_WM': 0,
            'DATA_READY': 0
        }

        # Create GUI
        self.create_widgets()

        # Start update loop
        self.update_data()

    def quat_to_euler(self, qw, qx, qy, qz):
        """
        Convert quaternion to Euler angles (ZYX convention - robotics)

        Args:
            qw, qx, qy, qz: Quaternion components

        Returns:
            (roll, pitch, yaw) in degrees
        """
        # Roll (x-axis rotation)
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2.0 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        # Convert to degrees
        return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)

    def decimate_data(self, time_array, data_array, max_points=10000):
        """
        Decimate data for plotting using Largest Triangle Three Buckets (LTTB) algorithm

        Args:
            time_array: numpy array of timestamps
            data_array: numpy array of data values
            max_points: maximum number of points to keep

        Returns:
            (decimated_time, decimated_data) tuple of numpy arrays
        """
        if len(time_array) <= max_points:
            return time_array, data_array

        # LTTB algorithm for visually accurate downsampling
        bucket_size = (len(time_array) - 2) / (max_points - 2)

        # Always keep first point
        decimated_indices = [0]

        for bucket_idx in range(max_points - 2):
            # Calculate bucket boundaries
            bucket_start = int(np.floor((bucket_idx + 0) * bucket_size)) + 1
            bucket_end = int(np.floor((bucket_idx + 1) * bucket_size)) + 1

            # Calculate average point of next bucket for reference
            next_bucket_start = int(np.floor((bucket_idx + 1) * bucket_size)) + 1
            next_bucket_end = min(int(np.floor((bucket_idx + 2) * bucket_size)) + 1, len(time_array))

            if next_bucket_end > next_bucket_start:
                avg_x = np.mean(time_array[next_bucket_start:next_bucket_end])
                avg_y = np.mean(data_array[next_bucket_start:next_bucket_end])
            else:
                avg_x = time_array[-1]
                avg_y = data_array[-1]

            # Find point in current bucket that forms largest triangle
            prev_idx = decimated_indices[-1]
            max_area = -1
            max_area_idx = bucket_start

            for idx in range(bucket_start, min(bucket_end, len(time_array))):
                # Calculate triangle area
                area = abs(
                    (time_array[prev_idx] - avg_x) * (data_array[idx] - data_array[prev_idx]) -
                    (time_array[prev_idx] - time_array[idx]) * (avg_y - data_array[prev_idx])
                )
                if area > max_area:
                    max_area = area
                    max_area_idx = idx

            decimated_indices.append(max_area_idx)

        # Always keep last point
        decimated_indices.append(len(time_array) - 1)

        return time_array[decimated_indices], data_array[decimated_indices]

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

        # ===== Data Management Controls (Second Row) =====
        data_control_frame = ttk.Frame(self.root, padding="5")
        data_control_frame.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E))

        # Clear/Reset Data button
        ttk.Button(data_control_frame, text="Clear Data",
                  command=self.clear_all_data).grid(row=0, column=0, padx=5)

        # Pause/Resume button
        self.pause_button = ttk.Button(data_control_frame, text="Pause",
                                       command=self.toggle_pause)
        self.pause_button.grid(row=0, column=1, padx=5)

        # Export CSV button
        ttk.Button(data_control_frame, text="Export CSV",
                  command=self.export_csv_data).grid(row=0, column=2, padx=5)

        # Export Plots button
        ttk.Button(data_control_frame, text="Export Plots",
                  command=self.export_plots).grid(row=0, column=3, padx=5)

        # ===== Create Notebook for Tabs =====
        notebook = ttk.Notebook(self.root)
        notebook.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), padx=5, pady=5)

        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(2, weight=1)  # Notebook is now in row 2

        # ===== Tab 1: IMU Data (6 separate plots) =====
        imu_frame = ttk.Frame(notebook)
        notebook.add(imu_frame, text="IMU Data")

        # Create matplotlib figure for IMU data (3 rows x 2 columns)
        self.fig_imu = Figure(figsize=(14, 10))

        # Accelerometer X (top left)
        self.ax_accel_x = self.fig_imu.add_subplot(3, 2, 1)
        self.ax_accel_x.set_title("Accelerometer X", fontsize=10)
        self.ax_accel_x.set_ylabel("Accel X (mg)")
        self.ax_accel_x.grid(True, alpha=0.3)
        self.line_accel_x, = self.ax_accel_x.plot([], [], 'r-', linewidth=1)

        # Gyroscope X (top right)
        self.ax_gyro_x = self.fig_imu.add_subplot(3, 2, 2)
        self.ax_gyro_x.set_title("Gyroscope X", fontsize=10)
        self.ax_gyro_x.set_ylabel("Gyro X (mdps)")
        self.ax_gyro_x.grid(True, alpha=0.3)
        self.line_gyro_x, = self.ax_gyro_x.plot([], [], 'r-', linewidth=1)

        # Accelerometer Y (middle left)
        self.ax_accel_y = self.fig_imu.add_subplot(3, 2, 3)
        self.ax_accel_y.set_title("Accelerometer Y", fontsize=10)
        self.ax_accel_y.set_ylabel("Accel Y (mg)")
        self.ax_accel_y.grid(True, alpha=0.3)
        self.line_accel_y, = self.ax_accel_y.plot([], [], 'g-', linewidth=1)

        # Gyroscope Y (middle right)
        self.ax_gyro_y = self.fig_imu.add_subplot(3, 2, 4)
        self.ax_gyro_y.set_title("Gyroscope Y", fontsize=10)
        self.ax_gyro_y.set_ylabel("Gyro Y (mdps)")
        self.ax_gyro_y.grid(True, alpha=0.3)
        self.line_gyro_y, = self.ax_gyro_y.plot([], [], 'g-', linewidth=1)

        # Accelerometer Z (bottom left)
        self.ax_accel_z = self.fig_imu.add_subplot(3, 2, 5)
        self.ax_accel_z.set_title("Accelerometer Z", fontsize=10)
        self.ax_accel_z.set_xlabel("Time (s)")
        self.ax_accel_z.set_ylabel("Accel Z (mg)")
        self.ax_accel_z.grid(True, alpha=0.3)
        self.line_accel_z, = self.ax_accel_z.plot([], [], 'b-', linewidth=1)

        # Gyroscope Z (bottom right)
        self.ax_gyro_z = self.fig_imu.add_subplot(3, 2, 6)
        self.ax_gyro_z.set_title("Gyroscope Z", fontsize=10)
        self.ax_gyro_z.set_xlabel("Time (s)")
        self.ax_gyro_z.set_ylabel("Gyro Z (mdps)")
        self.ax_gyro_z.grid(True, alpha=0.3)
        self.line_gyro_z, = self.ax_gyro_z.plot([], [], 'b-', linewidth=1)

        self.fig_imu.tight_layout()

        # Embed matplotlib in tkinter
        self.canvas_imu = FigureCanvasTkAgg(self.fig_imu, master=imu_frame)
        self.canvas_imu.draw()
        self.canvas_imu.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Add navigation toolbar for zoom/pan
        toolbar_imu = NavigationToolbar2Tk(self.canvas_imu, imu_frame)
        toolbar_imu.update()
        toolbar_imu.pack(side=tk.BOTTOM, fill=tk.X)

        # ===== Tab 2: Sensor Fusion (side-by-side layout) =====
        fusion_frame = ttk.Frame(notebook)
        notebook.add(fusion_frame, text="Sensor Fusion")

        # Create matplotlib figure for sensor fusion (4 rows x 2 columns)
        # Left column: Quaternions (W, X, Y, Z)
        # Right column: Euler angles (Roll, Pitch, Yaw)
        self.fig_fusion = Figure(figsize=(14, 10))

        # Left Column - Quaternion W (row 1, left)
        self.ax_quat_w = self.fig_fusion.add_subplot(4, 2, 1)
        self.ax_quat_w.set_title("Quaternion W", fontsize=10)
        self.ax_quat_w.set_ylabel("qW")
        self.ax_quat_w.grid(True, alpha=0.3)
        self.line_quat_w, = self.ax_quat_w.plot([], [], 'k-', linewidth=1)

        # Right Column - Roll angle (row 1, right)
        self.ax_roll = self.fig_fusion.add_subplot(4, 2, 2)
        self.ax_roll.set_title("Roll Angle", fontsize=10)
        self.ax_roll.set_ylabel("Roll (deg)")
        self.ax_roll.grid(True, alpha=0.3)
        self.line_roll, = self.ax_roll.plot([], [], 'm-', linewidth=1)

        # Left Column - Quaternion X (row 2, left)
        self.ax_quat_x = self.fig_fusion.add_subplot(4, 2, 3)
        self.ax_quat_x.set_title("Quaternion X", fontsize=10)
        self.ax_quat_x.set_ylabel("qX")
        self.ax_quat_x.grid(True, alpha=0.3)
        self.line_quat_x, = self.ax_quat_x.plot([], [], 'r-', linewidth=1)

        # Right Column - Pitch angle (row 2, right)
        self.ax_pitch = self.fig_fusion.add_subplot(4, 2, 4)
        self.ax_pitch.set_title("Pitch Angle", fontsize=10)
        self.ax_pitch.set_ylabel("Pitch (deg)")
        self.ax_pitch.grid(True, alpha=0.3)
        self.line_pitch, = self.ax_pitch.plot([], [], 'c-', linewidth=1)

        # Left Column - Quaternion Y (row 3, left)
        self.ax_quat_y = self.fig_fusion.add_subplot(4, 2, 5)
        self.ax_quat_y.set_title("Quaternion Y", fontsize=10)
        self.ax_quat_y.set_ylabel("qY")
        self.ax_quat_y.grid(True, alpha=0.3)
        self.line_quat_y, = self.ax_quat_y.plot([], [], 'g-', linewidth=1)

        # Right Column - Yaw angle (row 3, right)
        self.ax_yaw = self.fig_fusion.add_subplot(4, 2, 6)
        self.ax_yaw.set_title("Yaw Angle", fontsize=10)
        self.ax_yaw.set_xlabel("Time (s)")
        self.ax_yaw.set_ylabel("Yaw (deg)")
        self.ax_yaw.grid(True, alpha=0.3)
        self.line_yaw, = self.ax_yaw.plot([], [], 'y-', linewidth=1)

        # Left Column - Quaternion Z (row 4, left)
        self.ax_quat_z = self.fig_fusion.add_subplot(4, 2, 7)
        self.ax_quat_z.set_title("Quaternion Z", fontsize=10)
        self.ax_quat_z.set_xlabel("Time (s)")
        self.ax_quat_z.set_ylabel("qZ")
        self.ax_quat_z.grid(True, alpha=0.3)
        self.line_quat_z, = self.ax_quat_z.plot([], [], 'b-', linewidth=1)

        self.fig_fusion.tight_layout()

        # Embed matplotlib in tkinter
        self.canvas_fusion = FigureCanvasTkAgg(self.fig_fusion, master=fusion_frame)
        self.canvas_fusion.draw()
        self.canvas_fusion.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Add navigation toolbar for zoom/pan
        toolbar_fusion = NavigationToolbar2Tk(self.canvas_fusion, fusion_frame)
        toolbar_fusion.update()
        toolbar_fusion.pack(side=tk.BOTTOM, fill=tk.X)

        # ===== Tab 3: Configuration =====
        config_frame = ttk.Frame(notebook, padding="10")
        notebook.add(config_frame, text="Configuration")

        # Configure grid layout for config_frame (2 columns)
        config_frame.columnconfigure(0, weight=1)
        config_frame.columnconfigure(1, weight=1)

        # --- Panel 1: Accelerometer Basic (Row 0, Col 0) ---
        accel_basic_group = ttk.LabelFrame(config_frame, text="Accelerometer", padding="10")
        accel_basic_group.grid(row=0, column=0, padx=5, pady=5, sticky=(tk.W, tk.E, tk.N, tk.S))

        ttk.Label(accel_basic_group, text="ODR:").grid(row=0, column=0, sticky=tk.W, pady=3)
        self.acc_odr_var = tk.StringVar(value="120 Hz")
        ttk.Combobox(accel_basic_group, textvariable=self.acc_odr_var,
                     values=["1.875 Hz", "7.5 Hz", "15 Hz", "30 Hz", "60 Hz", "120 Hz",
                             "240 Hz", "480 Hz", "960 Hz", "1920 Hz", "3840 Hz", "7680 Hz",
                             "15.625 Hz (HA1)", "31.25 Hz (HA1)", "62.5 Hz (HA1)", "125 Hz (HA1)",
                             "250 Hz (HA1)", "500 Hz (HA1)", "1000 Hz (HA1)", "2000 Hz (HA1)",
                             "4000 Hz (HA1)", "8000 Hz (HA1)", "12.5 Hz (HA2)", "25 Hz (HA2)",
                             "50 Hz (HA2)", "100 Hz (HA2)", "200 Hz (HA2)", "400 Hz (HA2)",
                             "800 Hz (HA2)", "1600 Hz (HA2)", "3200 Hz (HA2)", "6400 Hz (HA2)"],
                     width=18, state="readonly").grid(row=0, column=1, pady=3, padx=5, sticky=(tk.W, tk.E))

        ttk.Label(accel_basic_group, text="Full Scale:").grid(row=1, column=0, sticky=tk.W, pady=3)
        self.acc_fs_var = tk.StringVar(value="±4g")
        ttk.Combobox(accel_basic_group, textvariable=self.acc_fs_var,
                     values=["±2g", "±4g", "±8g", "±16g"],
                     width=18, state="readonly").grid(row=1, column=1, pady=3, padx=5, sticky=(tk.W, tk.E))

        ttk.Label(accel_basic_group, text="Mode:").grid(row=2, column=0, sticky=tk.W, pady=3)
        self.acc_mode_var = tk.StringVar(value="High Performance")
        ttk.Combobox(accel_basic_group, textvariable=self.acc_mode_var,
                     values=["High Performance", "High Accuracy", "ODR Triggered",
                             "Low Power (2-avg)", "Low Power (4-avg)", "Low Power (8-avg)", "Normal"],
                     width=18, state="readonly").grid(row=2, column=1, pady=3, padx=5, sticky=(tk.W, tk.E))

        # --- Panel 2: Gyroscope Basic (Row 0, Col 1) ---
        gyro_basic_group = ttk.LabelFrame(config_frame, text="Gyroscope", padding="10")
        gyro_basic_group.grid(row=0, column=1, padx=5, pady=5, sticky=(tk.W, tk.E, tk.N, tk.S))

        ttk.Label(gyro_basic_group, text="ODR:").grid(row=0, column=0, sticky=tk.W, pady=3)
        self.gyro_odr_var = tk.StringVar(value="120 Hz")
        ttk.Combobox(gyro_basic_group, textvariable=self.gyro_odr_var,
                     values=["1.875 Hz", "7.5 Hz", "15 Hz", "30 Hz", "60 Hz", "120 Hz",
                             "240 Hz", "480 Hz", "960 Hz", "1920 Hz", "3840 Hz", "7680 Hz",
                             "15.625 Hz (HA1)", "31.25 Hz (HA1)", "62.5 Hz (HA1)", "125 Hz (HA1)",
                             "250 Hz (HA1)", "500 Hz (HA1)", "1000 Hz (HA1)", "2000 Hz (HA1)",
                             "4000 Hz (HA1)", "8000 Hz (HA1)", "12.5 Hz (HA2)", "25 Hz (HA2)",
                             "50 Hz (HA2)", "100 Hz (HA2)", "200 Hz (HA2)", "400 Hz (HA2)",
                             "800 Hz (HA2)", "1600 Hz (HA2)", "3200 Hz (HA2)", "6400 Hz (HA2)"],
                     width=18, state="readonly").grid(row=0, column=1, pady=3, padx=5, sticky=(tk.W, tk.E))

        ttk.Label(gyro_basic_group, text="Full Scale:").grid(row=1, column=0, sticky=tk.W, pady=3)
        self.gyro_fs_var = tk.StringVar(value="±2000 dps")
        ttk.Combobox(gyro_basic_group, textvariable=self.gyro_fs_var,
                     values=["±125 dps", "±250 dps", "±500 dps",
                             "±1000 dps", "±2000 dps", "±4000 dps"],
                     width=18, state="readonly").grid(row=1, column=1, pady=3, padx=5, sticky=(tk.W, tk.E))

        ttk.Label(gyro_basic_group, text="Mode:").grid(row=2, column=0, sticky=tk.W, pady=3)
        self.gyro_mode_var = tk.StringVar(value="High Performance")
        ttk.Combobox(gyro_basic_group, textvariable=self.gyro_mode_var,
                     values=["High Performance", "High Accuracy", "Sleep", "Low Power"],
                     width=18, state="readonly").grid(row=2, column=1, pady=3, padx=5, sticky=(tk.W, tk.E))

        # --- Panel 3: Accelerometer Filtering (Row 1, Col 0) ---
        accel_filter_group = ttk.LabelFrame(config_frame, text="Accelerometer Filtering", padding="10")
        accel_filter_group.grid(row=1, column=0, padx=5, pady=5, sticky=(tk.W, tk.E, tk.N, tk.S))

        self.xl_lpf2_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(accel_filter_group, text="LPF2 Enable",
                        variable=self.xl_lpf2_var).grid(row=0, column=0, sticky=tk.W, pady=3, columnspan=2)

        ttk.Label(accel_filter_group, text="LPF2 Bandwidth:").grid(row=1, column=0, sticky=tk.W, pady=3)
        self.xl_lpf2_bw_var = tk.StringVar(value="0")
        ttk.Combobox(accel_filter_group, textvariable=self.xl_lpf2_bw_var,
                     values=["0", "1", "2", "3", "4", "5", "6", "7"],
                     width=18, state="readonly").grid(row=1, column=1, pady=3, padx=5, sticky=(tk.W, tk.E))

        self.xl_hpf_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(accel_filter_group, text="HPF Enable",
                        variable=self.xl_hpf_var).grid(row=2, column=0, sticky=tk.W, pady=3, columnspan=2)

        self.xl_fast_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(accel_filter_group, text="Fast Settling",
                        variable=self.xl_fast_var).grid(row=3, column=0, sticky=tk.W, pady=3, columnspan=2)

        # --- Panel 4: Gyroscope Filtering (Row 1, Col 1) ---
        gyro_filter_group = ttk.LabelFrame(config_frame, text="Gyroscope Filtering", padding="10")
        gyro_filter_group.grid(row=1, column=1, padx=5, pady=5, sticky=(tk.W, tk.E, tk.N, tk.S))

        self.gy_lpf1_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(gyro_filter_group, text="LPF1 Enable",
                        variable=self.gy_lpf1_var).grid(row=0, column=0, sticky=tk.W, pady=3, columnspan=2)

        ttk.Label(gyro_filter_group, text="LPF1 Bandwidth:").grid(row=1, column=0, sticky=tk.W, pady=3)
        self.gy_lpf1_bw_var = tk.StringVar(value="0")
        ttk.Combobox(gyro_filter_group, textvariable=self.gy_lpf1_bw_var,
                     values=["0", "1", "2", "3", "4", "5", "6", "7"],
                     width=18, state="readonly").grid(row=1, column=1, pady=3, padx=5, sticky=(tk.W, tk.E))

        # --- Panel 5: SFLP (Row 2, Col 0) ---
        sflp_group = ttk.LabelFrame(config_frame, text="Sensor Fusion (SFLP)", padding="10")
        sflp_group.grid(row=2, column=0, padx=5, pady=5, sticky=(tk.W, tk.E, tk.N, tk.S))

        self.sflp_enabled_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(sflp_group, text="Enable SFLP",
                        variable=self.sflp_enabled_var,
                        command=self.toggle_sflp).grid(row=0, column=0, sticky=tk.W, pady=3, columnspan=2)

        ttk.Label(sflp_group, text="SFLP ODR:").grid(row=1, column=0, sticky=tk.W, pady=3)
        self.sflp_odr_var = tk.StringVar(value="15 Hz")
        ttk.Combobox(sflp_group, textvariable=self.sflp_odr_var,
                     values=["15 Hz", "30 Hz", "60 Hz", "120 Hz", "240 Hz", "480 Hz"],
                     width=18, state="readonly").grid(row=1, column=1, pady=3, padx=5, sticky=(tk.W, tk.E))

        # --- Control Buttons (Row 2, Col 1) ---
        button_group = ttk.LabelFrame(config_frame, text="Configuration Control", padding="10")
        button_group.grid(row=2, column=1, padx=5, pady=5, sticky=(tk.W, tk.E, tk.N, tk.S))

        ttk.Button(button_group, text="🔄 Refresh Config",
                   command=self.refresh_config).pack(fill=tk.X, pady=5, padx=5)

        ttk.Label(button_group, text="Apply all settings to device:",
                  font=("Arial", 9)).pack(pady=(10,5))
        ttk.Button(button_group, text="⚡ APPLY ALL ⚡",
                   command=self.apply_all_settings,
                   style="Accent.TButton").pack(fill=tk.X, pady=5, padx=5)

        # ===== Tab 4: Interrupt Events & Embedded Functions =====
        interrupt_frame = ttk.Frame(notebook, padding="5")
        notebook.add(interrupt_frame, text="Embedded Functions")

        # Create sub-notebook for interrupt functions
        int_notebook = ttk.Notebook(interrupt_frame)
        int_notebook.pack(fill=tk.BOTH, expand=True)

        # --- Sub-tab 1: Tap Detection ---
        tap_config_frame = ttk.Frame(int_notebook, padding="10")
        int_notebook.add(tap_config_frame, text="Tap Detection")

        # Enable/Disable
        tap_control_group = ttk.LabelFrame(tap_config_frame, text="Tap Control", padding="10")
        tap_control_group.grid(row=0, column=0, padx=10, pady=10, sticky=(tk.W, tk.E, tk.N))

        self.tap_enabled_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(tap_control_group, text="Enable Tap Detection", variable=self.tap_enabled_var,
                       command=lambda: self.send_command(f"{'ENABLE' if self.tap_enabled_var.get() else 'DISABLE'}:TAP")).grid(row=0, column=0, sticky=tk.W, pady=5, columnspan=2)

        # Thresholds
        tap_thresh_group = ttk.LabelFrame(tap_config_frame, text="Tap Thresholds (0-31)", padding="10")
        tap_thresh_group.grid(row=1, column=0, padx=10, pady=10, sticky=(tk.W, tk.E, tk.N))

        ttk.Label(tap_thresh_group, text="X Threshold:").grid(row=0, column=0, sticky=tk.W, pady=5)
        self.tap_thresh_x_var = tk.StringVar(value="15")
        ttk.Entry(tap_thresh_group, textvariable=self.tap_thresh_x_var, width=8).grid(row=0, column=1, pady=5)
        ttk.Button(tap_thresh_group, text="Apply", command=lambda: self.send_command(f"SET:TAP_THRESHOLD_X:{self.tap_thresh_x_var.get()}")).grid(row=0, column=2, padx=5)

        ttk.Label(tap_thresh_group, text="Y Threshold:").grid(row=1, column=0, sticky=tk.W, pady=5)
        self.tap_thresh_y_var = tk.StringVar(value="15")
        ttk.Entry(tap_thresh_group, textvariable=self.tap_thresh_y_var, width=8).grid(row=1, column=1, pady=5)
        ttk.Button(tap_thresh_group, text="Apply", command=lambda: self.send_command(f"SET:TAP_THRESHOLD_Y:{self.tap_thresh_y_var.get()}")).grid(row=1, column=2, padx=5)

        ttk.Label(tap_thresh_group, text="Z Threshold:").grid(row=2, column=0, sticky=tk.W, pady=5)
        self.tap_thresh_z_var = tk.StringVar(value="15")
        ttk.Entry(tap_thresh_group, textvariable=self.tap_thresh_z_var, width=8).grid(row=2, column=1, pady=5)
        ttk.Button(tap_thresh_group, text="Apply", command=lambda: self.send_command(f"SET:TAP_THRESHOLD_Z:{self.tap_thresh_z_var.get()}")).grid(row=2, column=2, padx=5)

        # Timing Parameters
        tap_timing_group = ttk.LabelFrame(tap_config_frame, text="Tap Timing", padding="10")
        tap_timing_group.grid(row=2, column=0, padx=10, pady=10, sticky=(tk.W, tk.E, tk.N))

        ttk.Label(tap_timing_group, text="Shock (0-3):").grid(row=0, column=0, sticky=tk.W, pady=5)
        self.tap_shock_var = tk.StringVar(value="2")
        ttk.Entry(tap_timing_group, textvariable=self.tap_shock_var, width=8).grid(row=0, column=1, pady=5)
        ttk.Button(tap_timing_group, text="Apply", command=lambda: self.send_command(f"SET:TAP_SHOCK:{self.tap_shock_var.get()}")).grid(row=0, column=2, padx=5)

        ttk.Label(tap_timing_group, text="Quiet (0-3):").grid(row=1, column=0, sticky=tk.W, pady=5)
        self.tap_quiet_var = tk.StringVar(value="2")
        ttk.Entry(tap_timing_group, textvariable=self.tap_quiet_var, width=8).grid(row=1, column=1, pady=5)
        ttk.Button(tap_timing_group, text="Apply", command=lambda: self.send_command(f"SET:TAP_QUIET:{self.tap_quiet_var.get()}")).grid(row=1, column=2, padx=5)

        ttk.Label(tap_timing_group, text="Latency (0-15):").grid(row=2, column=0, sticky=tk.W, pady=5)
        self.tap_latency_var = tk.StringVar(value="4")
        ttk.Entry(tap_timing_group, textvariable=self.tap_latency_var, width=8).grid(row=2, column=1, pady=5)
        ttk.Button(tap_timing_group, text="Apply", command=lambda: self.send_command(f"SET:TAP_LATENCY:{self.tap_latency_var.get()}")).grid(row=2, column=2, padx=5)

        # Axes and Mode
        tap_axes_group = ttk.LabelFrame(tap_config_frame, text="Tap Axes & Mode", padding="10")
        tap_axes_group.grid(row=3, column=0, padx=10, pady=10, sticky=(tk.W, tk.E, tk.N))

        self.tap_x_en_var = tk.BooleanVar(value=True)
        self.tap_y_en_var = tk.BooleanVar(value=True)
        self.tap_z_en_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(tap_axes_group, text="X Axis", variable=self.tap_x_en_var).grid(row=0, column=0, sticky=tk.W, pady=5)
        ttk.Checkbutton(tap_axes_group, text="Y Axis", variable=self.tap_y_en_var).grid(row=0, column=1, sticky=tk.W, pady=5)
        ttk.Checkbutton(tap_axes_group, text="Z Axis", variable=self.tap_z_en_var).grid(row=0, column=2, sticky=tk.W, pady=5)
        ttk.Button(tap_axes_group, text="Apply Axes",
                   command=lambda: self.send_command(f"SET:TAP_AXES:{'1' if self.tap_x_en_var.get() else '0'}{'1' if self.tap_y_en_var.get() else '0'}{'1' if self.tap_z_en_var.get() else '0'}")).grid(row=0, column=3, padx=5)

        ttk.Label(tap_axes_group, text="Mode:").grid(row=1, column=0, sticky=tk.W, pady=5)
        self.tap_mode_var = tk.StringVar(value="Single + Double")
        ttk.Combobox(tap_axes_group, textvariable=self.tap_mode_var,
                     values=["Single Only", "Single + Double"], width=15).grid(row=1, column=1, columnspan=2, pady=5)
        ttk.Button(tap_axes_group, text="Apply Mode",
                   command=lambda: self.send_command(f"SET:TAP_MODE:{0 if self.tap_mode_var.get() == 'Single Only' else 1}")).grid(row=1, column=3, padx=5)

        # --- Sub-tab 2: Wake-Up & Free Fall ---
        wake_ff_frame = ttk.Frame(int_notebook, padding="10")
        int_notebook.add(wake_ff_frame, text="Wake-Up & Free Fall")

        # Wake-Up
        wake_group = ttk.LabelFrame(wake_ff_frame, text="Wake-Up Detection", padding="10")
        wake_group.grid(row=0, column=0, padx=10, pady=10, sticky=(tk.W, tk.E, tk.N))

        self.wake_enabled_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(wake_group, text="Enable Wake-Up", variable=self.wake_enabled_var,
                       command=lambda: self.send_command(f"{'ENABLE' if self.wake_enabled_var.get() else 'DISABLE'}:WAKE_UP")).grid(row=0, column=0, sticky=tk.W, pady=5, columnspan=2)

        ttk.Label(wake_group, text="Threshold (0-63):").grid(row=1, column=0, sticky=tk.W, pady=5)
        self.wake_thresh_var = tk.StringVar(value="20")
        ttk.Entry(wake_group, textvariable=self.wake_thresh_var, width=8).grid(row=1, column=1, pady=5)
        ttk.Button(wake_group, text="Apply", command=lambda: self.send_command(f"SET:WAKE_THRESHOLD:{self.wake_thresh_var.get()}")).grid(row=1, column=2, padx=5)

        ttk.Label(wake_group, text="Duration (0-3):").grid(row=2, column=0, sticky=tk.W, pady=5)
        self.wake_dur_var = tk.StringVar(value="1")
        ttk.Entry(wake_group, textvariable=self.wake_dur_var, width=8).grid(row=2, column=1, pady=5)
        ttk.Button(wake_group, text="Apply", command=lambda: self.send_command(f"SET:WAKE_DURATION:{self.wake_dur_var.get()}")).grid(row=2, column=2, padx=5)

        self.wake_x_en_var = tk.BooleanVar(value=True)
        self.wake_y_en_var = tk.BooleanVar(value=True)
        self.wake_z_en_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(wake_group, text="X Axis", variable=self.wake_x_en_var).grid(row=3, column=0, sticky=tk.W, pady=5)
        ttk.Checkbutton(wake_group, text="Y Axis", variable=self.wake_y_en_var).grid(row=3, column=1, sticky=tk.W, pady=5)
        ttk.Checkbutton(wake_group, text="Z Axis", variable=self.wake_z_en_var).grid(row=3, column=2, sticky=tk.W, pady=5)
        ttk.Button(wake_group, text="Apply Axes",
                   command=lambda: self.send_command(f"SET:WAKE_AXES:{'1' if self.wake_x_en_var.get() else '0'}{'1' if self.wake_y_en_var.get() else '0'}{'1' if self.wake_z_en_var.get() else '0'}")).grid(row=4, column=0, columnspan=3, pady=5)

        # Free Fall
        ff_group = ttk.LabelFrame(wake_ff_frame, text="Free Fall Detection", padding="10")
        ff_group.grid(row=1, column=0, padx=10, pady=10, sticky=(tk.W, tk.E, tk.N))

        self.ff_enabled_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(ff_group, text="Enable Free Fall", variable=self.ff_enabled_var,
                       command=lambda: self.send_command(f"{'ENABLE' if self.ff_enabled_var.get() else 'DISABLE'}:FREE_FALL")).grid(row=0, column=0, sticky=tk.W, pady=5, columnspan=2)

        ttk.Label(ff_group, text="Threshold (0-7):").grid(row=1, column=0, sticky=tk.W, pady=5)
        self.ff_thresh_var = tk.StringVar(value="3")
        ttk.Combobox(ff_group, textvariable=self.ff_thresh_var,
                     values=["0 (156mg)", "1 (219mg)", "2 (250mg)", "3 (312mg)", "4 (344mg)", "5 (406mg)", "6 (469mg)", "7 (500mg)"], width=12).grid(row=1, column=1, pady=5)
        ttk.Button(ff_group, text="Apply", command=lambda: self.send_command(f"SET:FF_THRESHOLD:{self.ff_thresh_var.get().split()[0]}")).grid(row=1, column=2, padx=5)

        ttk.Label(ff_group, text="Duration (0-31):").grid(row=2, column=0, sticky=tk.W, pady=5)
        self.ff_dur_var = tk.StringVar(value="6")
        ttk.Entry(ff_group, textvariable=self.ff_dur_var, width=8).grid(row=2, column=1, pady=5)
        ttk.Button(ff_group, text="Apply", command=lambda: self.send_command(f"SET:FF_DURATION:{self.ff_dur_var.get()}")).grid(row=2, column=2, padx=5)

        # --- Sub-tab 3: 6D, Tilt, Motion ---
        motion_frame = ttk.Frame(int_notebook, padding="10")
        int_notebook.add(motion_frame, text="6D, Tilt & Motion")

        # 6D Orientation
        sixd_group = ttk.LabelFrame(motion_frame, text="6D Orientation", padding="10")
        sixd_group.grid(row=0, column=0, padx=10, pady=10, sticky=(tk.W, tk.E, tk.N))

        self.sixd_enabled_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(sixd_group, text="Enable 6D Detection", variable=self.sixd_enabled_var,
                       command=lambda: self.send_command(f"{'ENABLE' if self.sixd_enabled_var.get() else 'DISABLE'}:6D")).grid(row=0, column=0, sticky=tk.W, pady=5, columnspan=2)

        ttk.Label(sixd_group, text="Threshold:").grid(row=1, column=0, sticky=tk.W, pady=5)
        self.sixd_thresh_var = tk.StringVar(value="60")
        ttk.Combobox(sixd_group, textvariable=self.sixd_thresh_var,
                     values=["50", "60", "70", "80"], width=8).grid(row=1, column=1, pady=5)
        ttk.Button(sixd_group, text="Apply", command=lambda: self.send_command(f"SET:6D_THRESHOLD:{self.sixd_thresh_var.get()}")).grid(row=1, column=2, padx=5)

        # Tilt
        tilt_group = ttk.LabelFrame(motion_frame, text="Tilt Detection", padding="10")
        tilt_group.grid(row=1, column=0, padx=10, pady=10, sticky=(tk.W, tk.E, tk.N))

        self.tilt_enabled_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(tilt_group, text="Enable Tilt Detection", variable=self.tilt_enabled_var,
                       command=lambda: self.send_command(f"{'ENABLE' if self.tilt_enabled_var.get() else 'DISABLE'}:TILT")).grid(row=0, column=0, sticky=tk.W, pady=5)

        # Significant Motion
        sigmo_group = ttk.LabelFrame(motion_frame, text="Significant Motion", padding="10")
        sigmo_group.grid(row=2, column=0, padx=10, pady=10, sticky=(tk.W, tk.E, tk.N))

        self.sigmo_enabled_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(sigmo_group, text="Enable Significant Motion", variable=self.sigmo_enabled_var,
                       command=lambda: self.send_command(f"{'ENABLE' if self.sigmo_enabled_var.get() else 'DISABLE'}:SIG_MOTION")).grid(row=0, column=0, sticky=tk.W, pady=5)

        # --- Sub-tab 4: Step Counter & Calibration ---
        step_cal_frame = ttk.Frame(int_notebook, padding="10")
        int_notebook.add(step_cal_frame, text="Step & Cal")

        # Step Counter
        step_group = ttk.LabelFrame(step_cal_frame, text="Step Counter", padding="10")
        step_group.grid(row=0, column=0, padx=10, pady=10, sticky=(tk.W, tk.E, tk.N))

        self.step_enabled_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(step_group, text="Enable Step Counter", variable=self.step_enabled_var,
                       command=lambda: self.send_command(f"{'ENABLE' if self.step_enabled_var.get() else 'DISABLE'}:STEP_COUNTER")).grid(row=0, column=0, sticky=tk.W, pady=5, columnspan=2)

        self.step_count_var = tk.StringVar(value="0")
        ttk.Label(step_group, text="Step Count:").grid(row=1, column=0, sticky=tk.W, pady=5)
        ttk.Label(step_group, textvariable=self.step_count_var, font=("Arial", 14, "bold")).grid(row=1, column=1, sticky=tk.W, pady=5, padx=10)

        ttk.Button(step_group, text="Get Count", command=lambda: self.send_command("GET_STEP_COUNT")).grid(row=2, column=0, pady=5, padx=5)
        ttk.Button(step_group, text="Reset Counter", command=lambda: self.send_command("RESET_STEP_COUNT")).grid(row=2, column=1, pady=5, padx=5)

        # Self-Test
        selftest_group = ttk.LabelFrame(step_cal_frame, text="Self-Test", padding="10")
        selftest_group.grid(row=1, column=0, padx=10, pady=10, sticky=(tk.W, tk.E, tk.N))

        ttk.Button(selftest_group, text="Run Self-Test", command=lambda: self.send_command("SELF_TEST")).grid(row=0, column=0, pady=5, padx=5)

        self.selftest_result_var = tk.StringVar(value="Not tested")
        ttk.Label(selftest_group, text="Result:").grid(row=1, column=0, sticky=tk.W, pady=5)
        ttk.Label(selftest_group, textvariable=self.selftest_result_var).grid(row=1, column=1, sticky=tk.W, pady=5, padx=10)

        # Calibration
        cal_group = ttk.LabelFrame(step_cal_frame, text="Calibration", padding="10")
        cal_group.grid(row=2, column=0, padx=10, pady=10, sticky=(tk.W, tk.E, tk.N))

        ttk.Label(cal_group, text="Place sensor on flat surface").grid(row=0, column=0, columnspan=2, pady=5)
        ttk.Button(cal_group, text="Calibrate Offsets", command=lambda: self.send_command("CALIBRATE")).grid(row=1, column=0, columnspan=2, pady=5, padx=5)

        # --- Sub-tab 5: Event Log ---
        event_log_frame = ttk.Frame(int_notebook, padding="10")
        int_notebook.add(event_log_frame, text="Event Log")

        # Event Log
        log_group = ttk.LabelFrame(event_log_frame, text="Event Log", padding="10")
        log_group.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        self.interrupt_log = scrolledtext.ScrolledText(log_group, height=20, width=70,
                                                        wrap=tk.WORD, font=("Courier", 9))
        self.interrupt_log.pack(fill=tk.BOTH, expand=True)

        ttk.Button(log_group, text="Clear Log",
                  command=lambda: self.interrupt_log.delete(1.0, tk.END)).pack(pady=5)

        # Event Counters (in same tab)
        counter_group = ttk.LabelFrame(event_log_frame, text="Event Counters", padding="10")
        counter_group.pack(fill=tk.X, padx=10, pady=10)

        self.event_count_labels = {}
        counter_frame = ttk.Frame(counter_group)
        counter_frame.pack()

        col = 0
        row = 0
        for event in ['WAKE_UP', 'SINGLE_TAP', 'DOUBLE_TAP', 'FREE_FALL', '6D_ORIENT', 'TILT', 'STEP_DET', 'SIG_MOT']:
            ttk.Label(counter_frame, text=f"{event}:").grid(row=row, column=col*2, sticky=tk.W, pady=2, padx=5)
            label = ttk.Label(counter_frame, text="0", width=6)
            label.grid(row=row, column=col*2+1, sticky=tk.E, pady=2, padx=5)
            self.event_count_labels[event] = label
            col += 1
            if col >= 2:
                col = 0
                row += 1

        ttk.Button(counter_group, text="Reset All Counters",
                  command=self.reset_event_counters).pack(pady=10)

        # ===== Tab 5: Console/Log =====
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
        # Skip data collection if paused (but still parse for console logging)
        is_paused = self.paused

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

            # Check for step counter response (STEP_COUNT:value format)
            if line.startswith("STEP_COUNT:"):
                try:
                    step_count = line.split(':')[1].strip()
                    if hasattr(self, 'step_count_var'):
                        self.step_count_var.set(step_count)
                    self.log_message(f"Step count: {step_count}\n")
                except Exception as e:
                    self.log_message(f"ERROR parsing step count: {e}\n")
                return True

            # Check for self-test response (SELF_TEST:XL=PASS/FAIL,GY=PASS/FAIL format)
            if line.startswith("SELF_TEST:"):
                try:
                    result_str = line.split(':', 1)[1].strip()
                    if hasattr(self, 'selftest_result_var'):
                        self.selftest_result_var.set(result_str)
                    self.log_message(f"Self-test result: {result_str}\n")
                except Exception as e:
                    self.log_message(f"ERROR parsing self-test result: {e}\n")
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

                # Store data (only if not paused)
                if not is_paused:
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

                # Store quaternion data and timestamp (only if not paused)
                if not is_paused:
                    self.quat_time_data.append(timestamp)
                    self.quat_w.append(qw)
                    self.quat_x.append(qx)
                    self.quat_y.append(qy)
                    self.quat_z.append(qz)

                    # Compute and store Euler angles
                    roll, pitch, yaw = self.quat_to_euler(qw, qx, qy, qz)
                    self.euler_roll.append(roll)
                    self.euler_pitch.append(pitch)
                    self.euler_yaw.append(yaw)

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
            self.update_imu_plots()
            self.update_fusion_plots()

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

    def update_imu_plots(self):
        """Update IMU plots (accelerometer and gyroscope) with current data"""
        if len(self.time_data) == 0:
            return

        # Convert to numpy arrays for plotting
        time_array = np.array(self.time_data)
        time_array = time_array - time_array[0]  # Normalize to start at 0

        # Decimate data for performance
        time_dec_ax, accel_x_dec = self.decimate_data(time_array, np.array(self.accel_x))
        time_dec_ay, accel_y_dec = self.decimate_data(time_array, np.array(self.accel_y))
        time_dec_az, accel_z_dec = self.decimate_data(time_array, np.array(self.accel_z))
        time_dec_gx, gyro_x_dec = self.decimate_data(time_array, np.array(self.gyro_x))
        time_dec_gy, gyro_y_dec = self.decimate_data(time_array, np.array(self.gyro_y))
        time_dec_gz, gyro_z_dec = self.decimate_data(time_array, np.array(self.gyro_z))

        # Update accelerometer X plot
        self.line_accel_x.set_data(time_dec_ax, accel_x_dec)
        self.ax_accel_x.relim()
        self.ax_accel_x.autoscale_view(scalex=False, scaley=True)  # Only autoscale Y
        self.ax_accel_x.set_xlim(right=time_array[-1])  # Sticky right edge

        # Update accelerometer Y plot
        self.line_accel_y.set_data(time_dec_ay, accel_y_dec)
        self.ax_accel_y.relim()
        self.ax_accel_y.autoscale_view(scalex=False, scaley=True)
        self.ax_accel_y.set_xlim(right=time_array[-1])

        # Update accelerometer Z plot
        self.line_accel_z.set_data(time_dec_az, accel_z_dec)
        self.ax_accel_z.relim()
        self.ax_accel_z.autoscale_view(scalex=False, scaley=True)
        self.ax_accel_z.set_xlim(right=time_array[-1])

        # Update gyroscope X plot
        self.line_gyro_x.set_data(time_dec_gx, gyro_x_dec)
        self.ax_gyro_x.relim()
        self.ax_gyro_x.autoscale_view(scalex=False, scaley=True)
        self.ax_gyro_x.set_xlim(right=time_array[-1])

        # Update gyroscope Y plot
        self.line_gyro_y.set_data(time_dec_gy, gyro_y_dec)
        self.ax_gyro_y.relim()
        self.ax_gyro_y.autoscale_view(scalex=False, scaley=True)
        self.ax_gyro_y.set_xlim(right=time_array[-1])

        # Update gyroscope Z plot
        self.line_gyro_z.set_data(time_dec_gz, gyro_z_dec)
        self.ax_gyro_z.relim()
        self.ax_gyro_z.autoscale_view(scalex=False, scaley=True)
        self.ax_gyro_z.set_xlim(right=time_array[-1])

        # Redraw canvas
        self.canvas_imu.draw_idle()

    def update_fusion_plots(self):
        """Update sensor fusion plots (quaternions and Euler angles) with current data"""
        if len(self.quat_w) == 0:
            return

        # Convert to numpy arrays for plotting
        quat_time_array = np.array(self.quat_time_data)
        quat_time_array = quat_time_array - quat_time_array[0]  # Normalize to start at 0

        # Decimate data for performance
        time_dec_qw, quat_w_dec = self.decimate_data(quat_time_array, np.array(self.quat_w))
        time_dec_qx, quat_x_dec = self.decimate_data(quat_time_array, np.array(self.quat_x))
        time_dec_qy, quat_y_dec = self.decimate_data(quat_time_array, np.array(self.quat_y))
        time_dec_qz, quat_z_dec = self.decimate_data(quat_time_array, np.array(self.quat_z))
        time_dec_roll, euler_roll_dec = self.decimate_data(quat_time_array, np.array(self.euler_roll))
        time_dec_pitch, euler_pitch_dec = self.decimate_data(quat_time_array, np.array(self.euler_pitch))
        time_dec_yaw, euler_yaw_dec = self.decimate_data(quat_time_array, np.array(self.euler_yaw))

        # Update quaternion W plot
        self.line_quat_w.set_data(time_dec_qw, quat_w_dec)
        self.ax_quat_w.relim()
        self.ax_quat_w.autoscale_view(scalex=False, scaley=True)
        self.ax_quat_w.set_xlim(right=quat_time_array[-1])

        # Update quaternion X plot
        self.line_quat_x.set_data(time_dec_qx, quat_x_dec)
        self.ax_quat_x.relim()
        self.ax_quat_x.autoscale_view(scalex=False, scaley=True)
        self.ax_quat_x.set_xlim(right=quat_time_array[-1])

        # Update quaternion Y plot
        self.line_quat_y.set_data(time_dec_qy, quat_y_dec)
        self.ax_quat_y.relim()
        self.ax_quat_y.autoscale_view(scalex=False, scaley=True)
        self.ax_quat_y.set_xlim(right=quat_time_array[-1])

        # Update quaternion Z plot
        self.line_quat_z.set_data(time_dec_qz, quat_z_dec)
        self.ax_quat_z.relim()
        self.ax_quat_z.autoscale_view(scalex=False, scaley=True)
        self.ax_quat_z.set_xlim(right=quat_time_array[-1])

        # Update roll angle plot
        self.line_roll.set_data(time_dec_roll, euler_roll_dec)
        self.ax_roll.relim()
        self.ax_roll.autoscale_view(scalex=False, scaley=True)
        self.ax_roll.set_xlim(right=quat_time_array[-1])

        # Update pitch angle plot
        self.line_pitch.set_data(time_dec_pitch, euler_pitch_dec)
        self.ax_pitch.relim()
        self.ax_pitch.autoscale_view(scalex=False, scaley=True)
        self.ax_pitch.set_xlim(right=quat_time_array[-1])

        # Update yaw angle plot
        self.line_yaw.set_data(time_dec_yaw, euler_yaw_dec)
        self.ax_yaw.relim()
        self.ax_yaw.autoscale_view(scalex=False, scaley=True)
        self.ax_yaw.set_xlim(right=quat_time_array[-1])

        # Redraw canvas
        self.canvas_fusion.draw_idle()

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
        time.sleep(0.02)  # 20ms delay to allow MCU to process command

        # Parse and send accelerometer full-scale
        acc_fs = self.acc_fs_var.get().replace('±', '').replace('g', 'G')  # "±4g" -> "4G"
        self.send_command(f"SET:ACC_FS:{acc_fs}")
        time.sleep(0.02)

        # Parse and send gyro ODR
        gyro_odr = self.gyro_odr_var.get().split()[0]  # "120 Hz" -> "120"
        self.send_command(f"SET:GYRO_ODR:{gyro_odr}")
        time.sleep(0.02)

        # Parse and send gyro full-scale
        gyro_fs = self.gyro_fs_var.get().replace('±', '').replace(' dps', 'DPS')  # "±2000 dps" -> "2000DPS"
        self.send_command(f"SET:GYRO_FS:{gyro_fs}")

        self.log_message("Configuration commands sent\n")

    def toggle_sflp(self):
        """Toggle SFLP enable"""
        enabled = self.sflp_enabled_var.get()
        if enabled:
            self.send_command(f"ENABLE:SFLP")
            time.sleep(0.02)  # 20ms delay to allow MCU to process command
            sflp_odr = self.sflp_odr_var.get().split()[0]  # "15 Hz" -> "15"
            self.send_command(f"SET:SFLP_ODR:{sflp_odr}")
        else:
            self.send_command(f"DISABLE:SFLP")
        self.log_message(f"SFLP {'Enabled' if enabled else 'Disabled'}\n")

    def refresh_config(self):
        """Query current configuration from firmware"""
        self.log_message("=" * 60 + "\n")
        self.log_message("REFRESHING CONFIGURATION FROM DEVICE\n")
        self.log_message("=" * 60 + "\n")
        self.send_command("GET:CONFIG")
        self.log_message("Configuration query sent. Check console for device response.\n")
        self.log_message("Note: GUI controls are NOT automatically updated from device response.\n")
        self.log_message("=" * 60 + "\n")

    def apply_odr_with_mode_check(self, sensor_type):
        """Apply ODR with automatic High Accuracy mode switching if needed"""
        if sensor_type == 'ACC':
            odr_value = self.acc_odr_var.get()
            mode_var = self.acc_mode_var
            mode_sensor_type = 'ACC'
        else:  # GYRO
            odr_value = self.gyro_odr_var.get()
            mode_var = self.gyro_mode_var
            mode_sensor_type = 'GYRO'

        # Check if HA mode ODR is selected
        if '(HA1)' in odr_value or '(HA2)' in odr_value:
            # Automatically switch to High Accuracy mode
            if mode_var.get() != "High Accuracy":
                self.log_message(f"HA mode ODR selected - automatically switching {sensor_type} to High Accuracy mode\n")
                mode_var.set("High Accuracy")
                # Send the mode command first
                if sensor_type == 'ACC':
                    self.send_command("SET:ACC_MODE:1")
                else:
                    self.send_command("SET:GYRO_MODE:1")
                time.sleep(0.02)

        # Send the ODR command
        odr_numeric = odr_value.split()[0]  # Extract just the numeric part
        self.send_command(f"SET:{sensor_type}_ODR:{odr_numeric}")
        self.log_message(f"{sensor_type} ODR set to {odr_value}\n")

    def apply_all_settings(self):
        """Apply all configuration settings across all tabs"""
        self.log_message("=" * 60 + "\n")
        self.log_message("APPLYING ALL CONFIGURATION SETTINGS\n")
        self.log_message("=" * 60 + "\n")

        # Basic Configuration
        self.log_message("\n[1/6] Applying Basic Configuration...\n")

        # Accelerometer settings with HA mode check
        self.apply_odr_with_mode_check('ACC')
        time.sleep(0.02)

        acc_fs = self.acc_fs_var.get()[1:-1]  # "±4g" -> "4g"
        self.send_command(f"SET:ACC_FS:{acc_fs}")
        time.sleep(0.02)

        # Gyroscope settings with HA mode check
        self.apply_odr_with_mode_check('GYRO')
        time.sleep(0.02)

        gyro_fs = self.gyro_fs_var.get().split()[0][1:]  # "±2000 dps" -> "2000"
        self.send_command(f"SET:GYRO_FS:{gyro_fs}")
        time.sleep(0.02)

        # Power Modes (if not already set by HA mode check)
        self.log_message("\n[2/6] Applying Power Modes...\n")

        acc_mode_map = {'High Performance': '0', 'High Accuracy': '1', 'ODR Triggered': '3',
                        'Low Power (2-avg)': '4', 'Low Power (4-avg)': '5', 'Low Power (8-avg)': '6', 'Normal': '7'}
        acc_mode = acc_mode_map.get(self.acc_mode_var.get(), '0')
        self.send_command(f"SET:ACC_MODE:{acc_mode}")
        time.sleep(0.02)

        gyro_mode_map = {'High Performance': '0', 'High Accuracy': '1', 'Sleep': '4', 'Low Power': '5'}
        gyro_mode = gyro_mode_map.get(self.gyro_mode_var.get(), '0')
        self.send_command(f"SET:GYRO_MODE:{gyro_mode}")
        time.sleep(0.02)

        # Hardware Filtering
        self.log_message("\n[3/6] Applying Hardware Filtering...\n")

        self.send_command(f"SET:XL_LPF2:{1 if self.xl_lpf2_var.get() else 0}")
        time.sleep(0.02)
        self.send_command(f"SET:XL_LPF2_BW:{self.xl_lpf2_bw_var.get()}")
        time.sleep(0.02)
        self.send_command(f"SET:XL_HPF:{1 if self.xl_hpf_var.get() else 0}")
        time.sleep(0.02)
        self.send_command(f"SET:XL_FAST_SETTLING:{1 if self.xl_fast_var.get() else 0}")
        time.sleep(0.02)
        self.send_command(f"SET:GY_LPF1:{1 if self.gy_lpf1_var.get() else 0}")
        time.sleep(0.02)
        self.send_command(f"SET:GY_LPF1_BW:{self.gy_lpf1_bw_var.get()}")
        time.sleep(0.02)

        # SFLP Configuration
        self.log_message("\n[4/6] Applying SFLP Configuration...\n")

        if self.sflp_enabled_var.get():
            self.send_command("ENABLE:SFLP")
            time.sleep(0.02)
            sflp_odr = self.sflp_odr_var.get().split()[0]
            self.send_command(f"SET:SFLP_ODR:{sflp_odr}")
            time.sleep(0.02)
        else:
            self.send_command("DISABLE:SFLP")
            time.sleep(0.02)

        # Embedded Functions - Tap Detection
        self.log_message("\n[5/6] Applying Embedded Functions (Tap, Wake-Up, etc.)...\n")

        if self.tap_enabled_var.get():
            self.send_command("ENABLE:TAP")
            time.sleep(0.02)
            self.send_command(f"SET:TAP_THRESHOLD_X:{self.tap_thresh_x_var.get()}")
            time.sleep(0.02)
            self.send_command(f"SET:TAP_THRESHOLD_Y:{self.tap_thresh_y_var.get()}")
            time.sleep(0.02)
            self.send_command(f"SET:TAP_THRESHOLD_Z:{self.tap_thresh_z_var.get()}")
            time.sleep(0.02)
            self.send_command(f"SET:TAP_SHOCK:{self.tap_shock_var.get()}")
            time.sleep(0.02)
            self.send_command(f"SET:TAP_QUIET:{self.tap_quiet_var.get()}")
            time.sleep(0.02)
            self.send_command(f"SET:TAP_LATENCY:{self.tap_latency_var.get()}")
            time.sleep(0.02)
            tap_axes = f"{'1' if self.tap_x_en_var.get() else '0'}{'1' if self.tap_y_en_var.get() else '0'}{'1' if self.tap_z_en_var.get() else '0'}"
            self.send_command(f"SET:TAP_AXES:{tap_axes}")
            time.sleep(0.02)
            tap_mode = 0 if self.tap_mode_var.get() == 'Single Only' else 1
            self.send_command(f"SET:TAP_MODE:{tap_mode}")
            time.sleep(0.02)
        else:
            self.send_command("DISABLE:TAP")
            time.sleep(0.02)

        # Wake-Up Detection
        if self.wake_enabled_var.get():
            self.send_command("ENABLE:WAKE_UP")
            time.sleep(0.02)
            self.send_command(f"SET:WAKE_THRESHOLD:{self.wake_thresh_var.get()}")
            time.sleep(0.02)
            self.send_command(f"SET:WAKE_DURATION:{self.wake_dur_var.get()}")
            time.sleep(0.02)
            wake_axes = f"{'1' if self.wake_x_en_var.get() else '0'}{'1' if self.wake_y_en_var.get() else '0'}{'1' if self.wake_z_en_var.get() else '0'}"
            self.send_command(f"SET:WAKE_AXES:{wake_axes}")
            time.sleep(0.02)
        else:
            self.send_command("DISABLE:WAKE_UP")
            time.sleep(0.02)

        # Free Fall Detection
        if self.ff_enabled_var.get():
            self.send_command("ENABLE:FREE_FALL")
            time.sleep(0.02)
            ff_thresh = self.ff_thresh_var.get().split()[0]
            self.send_command(f"SET:FF_THRESHOLD:{ff_thresh}")
            time.sleep(0.02)
            self.send_command(f"SET:FF_DURATION:{self.ff_dur_var.get()}")
            time.sleep(0.02)
        else:
            self.send_command("DISABLE:FREE_FALL")
            time.sleep(0.02)

        # 6D Orientation
        if self.sixd_enabled_var.get():
            self.send_command("ENABLE:6D")
            time.sleep(0.02)
            self.send_command(f"SET:6D_THRESHOLD:{self.sixd_thresh_var.get()}")
            time.sleep(0.02)
        else:
            self.send_command("DISABLE:6D")
            time.sleep(0.02)

        # Tilt Detection
        if self.tilt_enabled_var.get():
            self.send_command("ENABLE:TILT")
            time.sleep(0.02)
        else:
            self.send_command("DISABLE:TILT")
            time.sleep(0.02)

        # Significant Motion
        if self.sigmo_enabled_var.get():
            self.send_command("ENABLE:SIG_MOTION")
            time.sleep(0.02)
        else:
            self.send_command("DISABLE:SIG_MOTION")
            time.sleep(0.02)

        # Step Counter
        if self.step_enabled_var.get():
            self.send_command("ENABLE:STEP_COUNTER")
            time.sleep(0.02)
        else:
            self.send_command("DISABLE:STEP_COUNTER")
            time.sleep(0.02)

        self.log_message("\n[6/6] Configuration Complete!\n")
        self.log_message("=" * 60 + "\n")
        self.log_message("ALL SETTINGS APPLIED SUCCESSFULLY\n")
        self.log_message("=" * 60 + "\n")

    def reset_event_counters(self):
        """Reset all interrupt event counters"""
        for event in self.event_counters.keys():
            self.event_counters[event] = 0

        # Update display
        for event, label in self.event_count_labels.items():
            label.config(text="0")

        self.log_message("Event counters reset\n")

    def clear_all_data(self):
        """Clear all data buffers and reset statistics"""
        # Clear all data deques
        self.time_data.clear()
        self.accel_x.clear()
        self.accel_y.clear()
        self.accel_z.clear()
        self.gyro_x.clear()
        self.gyro_y.clear()
        self.gyro_z.clear()
        self.quat_time_data.clear()
        self.quat_w.clear()
        self.quat_x.clear()
        self.quat_y.clear()
        self.quat_z.clear()
        self.euler_roll.clear()
        self.euler_pitch.clear()
        self.euler_yaw.clear()

        # Reset statistics
        self.sample_count = 0
        self.parse_error_count = 0
        self.empty_field_count = 0
        self.start_time = time.time()

        self.log_message("All data cleared\n")

    def toggle_pause(self):
        """Toggle pause/resume data collection"""
        self.paused = not self.paused
        if self.paused:
            self.pause_button.config(text="Resume")
            self.log_message("Data collection paused\n")
        else:
            self.pause_button.config(text="Pause")
            self.log_message("Data collection resumed\n")

    def export_csv_data(self):
        """Export all collected data to CSV file"""
        if len(self.time_data) == 0:
            messagebox.showwarning("No Data", "No data to export. Please collect some data first.")
            return

        # Generate default filename with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        default_filename = f"LSM6DSV_data_{timestamp}.csv"

        # Ask user for save location
        filename = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
            initialfile=default_filename
        )

        if not filename:
            return  # User cancelled

        try:
            with open(filename, 'w') as f:
                # Write header
                f.write("timestamp_us,accel_x_mg,accel_y_mg,accel_z_mg,gyro_x_mdps,gyro_y_mdps,gyro_z_mdps")

                # Add SFLP columns if data exists
                if len(self.quat_w) > 0:
                    f.write(",quat_timestamp_us,quat_w,quat_x,quat_y,quat_z,euler_roll_deg,euler_pitch_deg,euler_yaw_deg")

                f.write("\n")

                # Write IMU data
                for i in range(len(self.time_data)):
                    # Convert timestamp back to microseconds
                    timestamp_us = int(self.time_data[i] * 1000000)
                    f.write(f"{timestamp_us},{self.accel_x[i]},{self.accel_y[i]},{self.accel_z[i]},"
                           f"{self.gyro_x[i]},{self.gyro_y[i]},{self.gyro_z[i]}")

                    # Add SFLP data if available for this index
                    if i < len(self.quat_w):
                        quat_timestamp_us = int(self.quat_time_data[i] * 1000000)
                        f.write(f",{quat_timestamp_us},{self.quat_w[i]},{self.quat_x[i]},{self.quat_y[i]},{self.quat_z[i]},"
                               f"{self.euler_roll[i]},{self.euler_pitch[i]},{self.euler_yaw[i]}")

                    f.write("\n")

            messagebox.showinfo("Export Successful", f"Data exported to:\n{filename}\n\n{len(self.time_data)} samples saved.")
            self.log_message(f"Data exported to {filename} ({len(self.time_data)} samples)\n")

        except Exception as e:
            messagebox.showerror("Export Error", f"Failed to export data:\n{str(e)}")
            self.log_message(f"Export error: {str(e)}\n")

    def export_plots(self):
        """Export all plots as PNG images to a folder"""
        if len(self.time_data) == 0 and len(self.quat_w) == 0:
            messagebox.showwarning("No Data", "No data to plot. Please collect some data first.")
            return

        # Generate folder name with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        folder_name = f"LSM6DSV_plots_{timestamp}"

        # Ask user for directory
        directory = filedialog.askdirectory(title="Select folder to save plots")

        if not directory:
            return  # User cancelled

        try:
            # Create subfolder for plots
            plot_folder = os.path.join(directory, folder_name)
            os.makedirs(plot_folder, exist_ok=True)

            plot_count = 0

            # Save IMU plots
            if len(self.time_data) > 0:
                # Save individual IMU plots
                for ax, name in [
                    (self.ax_accel_x, "accel_x"),
                    (self.ax_accel_y, "accel_y"),
                    (self.ax_accel_z, "accel_z"),
                    (self.ax_gyro_x, "gyro_x"),
                    (self.ax_gyro_y, "gyro_y"),
                    (self.ax_gyro_z, "gyro_z")
                ]:
                    extent = ax.get_window_extent().transformed(self.fig_imu.dpi_scale_trans.inverted())
                    self.fig_imu.savefig(os.path.join(plot_folder, f"{name}.png"),
                                        bbox_inches=extent.expanded(1.3, 1.3), dpi=300)
                    plot_count += 1

                # Save full IMU figure
                self.fig_imu.savefig(os.path.join(plot_folder, "imu_full.png"), dpi=300, bbox_inches='tight')
                plot_count += 1

            # Save Fusion plots
            if len(self.quat_w) > 0:
                # Save individual fusion plots
                for ax, name in [
                    (self.ax_quat_w, "quat_w"),
                    (self.ax_quat_x, "quat_x"),
                    (self.ax_quat_y, "quat_y"),
                    (self.ax_quat_z, "quat_z"),
                    (self.ax_roll, "euler_roll"),
                    (self.ax_pitch, "euler_pitch"),
                    (self.ax_yaw, "euler_yaw")
                ]:
                    extent = ax.get_window_extent().transformed(self.fig_fusion.dpi_scale_trans.inverted())
                    self.fig_fusion.savefig(os.path.join(plot_folder, f"{name}.png"),
                                           bbox_inches=extent.expanded(1.3, 1.3), dpi=300)
                    plot_count += 1

                # Save full fusion figure
                self.fig_fusion.savefig(os.path.join(plot_folder, "fusion_full.png"), dpi=300, bbox_inches='tight')
                plot_count += 1

            messagebox.showinfo("Export Successful", f"Plots exported to:\n{plot_folder}\n\n{plot_count} images saved.")
            self.log_message(f"Plots exported to {plot_folder} ({plot_count} images)\n")

        except Exception as e:
            messagebox.showerror("Export Error", f"Failed to export plots:\n{str(e)}")
            self.log_message(f"Plot export error: {str(e)}\n")

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
