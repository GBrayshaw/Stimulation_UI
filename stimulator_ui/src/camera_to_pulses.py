import tkinter as tk
from tkinter import messagebox, filedialog

from metavision_sdk_core import PolarityFilterAlgorithm
from metavision_sdk_stream import Camera, CameraStreamSlicer
import numpy as np
from collections import deque
import serial
import threading
import time
import queue
import struct
from gpiozero import DigitalOutputDevice, DigitalInputDevice
import uart_protocol
import json

from cri.robot import SyncRobot, AsyncRobot
from cri.controller import RTDEController 
from cri.controller import MG400Controller as Controller

MODE = uart_protocol.MODE
PAYLOAD_TYPE = uart_protocol.PAYLOAD_TYPE
UART_START_BYTE = uart_protocol.UART_START_BYTE
UART_END_BYTE = uart_protocol.UART_END_BYTE


class IO2Pulse:
    def __init__(self, serial_port: str = "/dev/ttyUSB0", baudrate: int = uart_protocol.CONTROL_BOARD_UART_BAUD_RATE):
        # Triggers out
        # GPIO27 & GND (Pin 13 and 14)
        # Triggers in
        # GPIO13 and GND (Pin 33 and 34)
        self.trigger_out = DigitalOutputDevice(pin="GPIO27", active_high=True, initial_value=False)
        self.trigger_in = DigitalInputDevice(pin="GPIO13", pull_up=False, active_state=None) #TODO: Set to False when wired in

        # Camera objects (connected via connect_camera)
        self.camera = None
        self.slicer = None

        # Trigger mode and safety state, controlled via UART messages from peer Pi
        # use_external_triggers == True means this class should drive triggers
        self.use_external_triggers = False  # default to internal triggers
        self.in_safe_state = False
        self._awaiting_ack = False

        # Stimulation parameters
        self.pulse_width = 0            # in microseconds
        self.stim_amplitude = 0.0       # in appropriate current units (e.g. uA)
        self.frequency = 0              # in Hz
        self.low_frequency = 200    # Hardcoded frequencies of stimulation 
        self.mid_frequency = 500
        self.high_frequency = 1000

        # Spike encoding parameters
        self.high_spike_threshold = 20000  # Spike rates for trigger logic
        self.mid_spike_threshold = 10000
        self.low_spike_threshold = 5000
        self.time_window_us = 10000   # Time window for spike counting in microseconds
        # Sliding time-window spike counting state
        self._window_start_ts = None   # start timestamp (µs) of current counting window
        self._events_in_window = 0     # number of events in current window

        # Internal log queue for cross-thread status messages
        self._log_queue = queue.Queue()

        # Serial link config to the other Pi over USB (connected via connect_pi)
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.ser = None

        # Thread control
        self._stop_event = threading.Event()
        self._state_lock = threading.Lock()

        # UART listener thread for inter-Pi state messages
        self._uart_thread = None

        # Input pulse feedback logging thread
        self.timestamps_us = deque()
        self.levels = deque()
        self.window_size_us = 5_000_000  # 5 seconds in microseconds
        self._timestamp_thread = threading.Thread(target=self.trigger_in_thread, daemon=True)
        self._timestamp_thread.start()

        # Event camera processing thread (run loop)
        self._camera_thread = None

    # Logging trigger input state changes
    def trigger_in_thread(self):
        """Background thread: record microsecond timestamps on every input edge.

        Uses gpiozero's edge waiting to avoid busy-waiting, and stores a rolling
        window of (timestamp_us, level) samples.
        """
        last_state = int(self.trigger_in.value)
        t_us = time.monotonic_ns() // 1000
        self.timestamps_us.append(t_us)
        self.levels.append(last_state)

        while not self._stop_event.is_set():
            # Wait for the next edge (high->low or low->high)
            if last_state:
                self.trigger_in.wait_for_inactive(timeout=1.0)
            else:
                self.trigger_in.wait_for_active(timeout=1.0)

            if self._stop_event.is_set():
                break

            # Record timestamp at the moment the edge is detected
            t_us = time.monotonic_ns() // 1000
            state = int(self.trigger_in.value)

            self.timestamps_us.append(t_us)
            self.levels.append(state)

            # Maintain rolling window
            cutoff = t_us - self.window_size_us
            while self.timestamps_us and self.timestamps_us[0] < cutoff:
                self.timestamps_us.popleft()
                self.levels.popleft()

            last_state = state

    # ----------------------------
    # UART framing and state handling
    # ----------------------------
    def _compute_crc(self, msg_type: int, payload_type: int, payload_bytes: bytes) -> int:
        crc = msg_type ^ payload_type ^ len(payload_bytes)
        for b in payload_bytes:
            crc ^= b
        return crc & 0xFF

    # ----------------------------
    # Logging helpers
    # ----------------------------
    def log_message(self, message: str):
        """Enqueue a log message for consumption by the UI thread.

        Thread-safe: called from worker threads.
        """
        try:
            self._log_queue.put_nowait(message)
        except Exception:
            pass

    def get_log_messages(self, max_items: int = 50):
        """Retrieve up to max_items log messages without blocking."""
        msgs = []
        for _ in range(max_items):
            try:
                msgs.append(self._log_queue.get_nowait())
            except queue.Empty:
                break
            except Exception:
                break
        return msgs

    def _read_packet(self, timeout_s: float = 0.5):
        # If serial link is not available, wait briefly and return None
        if not self.ser or not getattr(self.ser, "is_open", False):
            time.sleep(min(timeout_s, 0.5))
            return None

        deadline = time.monotonic() + max(0.0, timeout_s)
        while time.monotonic() < deadline and not self._stop_event.is_set():
            try:
                start = self.ser.read(1)
            except serial.SerialException:
                # Serial link broke; mark as disconnected and exit
                self.log_message("Serial error while reading: closing link.")
                try:
                    if self.ser and self.ser.is_open:
                        self.ser.close()
                except Exception:
                    pass
                self.ser = None
                return None

            if not start or start[0] != UART_START_BYTE:
                continue

            header = self.ser.read(3)
            if len(header) != 3:
                continue
            msg_type, payload_type, length = header

            payload = self.ser.read(length)
            if len(payload) != length:
                continue

            tail = self.ser.read(2)
            if len(tail) != 2:
                continue
            crc_rx, end = tail
            if end != UART_END_BYTE:
                continue

            if self._compute_crc(msg_type, payload_type, payload) != crc_rx:
                continue

            return msg_type, payload_type, payload
        return None

    def _uart_loop(self):
        """Background thread: listen for state messages from the peer Pi.

        Handles:
        - MODE.UART_MSG_SHUTDOWN: enter safe state and wait for ACK
        - MODE.UART_MSG_TRIG_EXTERNAL: select external trigger mode (use this class)
        - MODE.UART_MSG_TRIG_INTERNAL: select internal trigger mode
        - MODE.UART_MSG_ACK: clear safe state after shutdown
        - MODE.UART_MSG_UPDATE_WIDTH: set internal pulse_width
        - MODE.UART_MSG_UPDATE_AMP: set internal stim_amplitude
        """
        while not self._stop_event.is_set():
            pkt = self._read_packet(timeout_s=0.5)
            if not pkt:
                continue

            msg_type, payload_type, payload = pkt
            with self._state_lock:
                if msg_type == MODE.UART_MSG_SHUTDOWN:
                    # Enter safe state and wait for ACK from the other Pi
                    self.in_safe_state = True
                    self._awaiting_ack = True
                    self.log_message("Received SHUTDOWN: entering safe state and awaiting ACK.")
                elif msg_type == MODE.UART_MSG_TRIG_EXTERNAL:
                    self.use_external_triggers = True
                    self.log_message("Received TRIG_EXTERNAL: enabling external triggers.")
                elif msg_type == MODE.UART_MSG_TRIG_INTERNAL:
                    self.use_external_triggers = False
                     # remain in current safe-state setting
                    self.log_message("Received TRIG_INTERNAL: disabling external triggers.")
                elif msg_type == MODE.UART_MSG_ACK and self._awaiting_ack:
                    # ACK received: exit the shutdown safe-wait state
                    self._awaiting_ack = False
                    self.in_safe_state = False
                    self.log_message("Received ACK: leaving safe shutdown state.")
                elif msg_type == MODE.UART_MSG_UPDATE_WIDTH:
                    width = None
                    try:
                        if payload_type == PAYLOAD_TYPE.PAYLOAD_INT32 and payload and len(payload) == 4:
                            width = int.from_bytes(payload, byteorder="big", signed=True)
                        elif payload_type == PAYLOAD_TYPE.PAYLOAD_BYTES and payload:
                            b = bytes(payload[:4]).ljust(4, b"\x00")
                            width = int.from_bytes(b, byteorder="big", signed=True)
                    except Exception:
                        width = None
                    if width is not None:
                        self.pulse_width = width
                        self.log_message(f"Received UPDATE_WIDTH: pulse_width set to {width}.")
                elif msg_type == MODE.UART_MSG_UPDATE_AMP:
                    amp = None
                    try:
                        if payload_type == PAYLOAD_TYPE.PAYLOAD_FLOAT32 and payload and len(payload) == 4:
                            amp = struct.unpack('>f', payload)[0]
                        elif payload_type == PAYLOAD_TYPE.PAYLOAD_BYTES and payload and len(payload) >= 4:
                            amp = struct.unpack('>f', bytes(payload[:4]))[0]
                    except Exception:
                        amp = None
                    if amp is not None:
                        self.stim_amplitude = float(amp)
                        self.log_message(f"Received UPDATE_AMP: stim_amplitude set to {self.stim_amplitude:.3f}.")

    def is_safe(self) -> bool:
        with self._state_lock:
            return self.in_safe_state

    def using_external_triggers(self) -> bool:
        with self._state_lock:
            return self.use_external_triggers

    def get_pulse_width(self) -> int:
        with self._state_lock:
            return int(self.pulse_width)

    def get_stim_amplitude(self) -> float:
        with self._state_lock:
            return float(self.stim_amplitude)

    def get_frequency(self) -> int:
        with self._state_lock:
            return int(self.frequency)

    def set_frequency(self, freq: int):
        """Setter for frequency that also generates a log entry.

        This is intended to drive the UI indicator via the regular
        _update_state_labels polling loop in CameraIOApp.
        """
        with self._state_lock:
            self.frequency = int(freq)
            current = self.frequency
        # Log outside the lock
        self.log_message(f"Frequency updated to {current} Hz.")

    # Connection status helpers
    def is_camera_connected(self) -> bool:
        return (self.camera is not None) and (self.slicer is not None)

    def is_pi_connected(self) -> bool:
        return (self.ser is not None) and getattr(self.ser, "is_open", False)

    def stop(self):
        """Signal the background thread to stop and wait for it to finish."""
        self._stop_event.set()
        try:
            if self._timestamp_thread:
                self._timestamp_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            if self._uart_thread:
                self._uart_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            if self._camera_thread:
                self._camera_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        try:
            if self.trigger_out:
                self.trigger_out.close()
        except Exception:
            pass
        try:
            if self.trigger_in:
                self.trigger_in.close()
        except Exception:
            pass

    # Connection helpers
    def connect_camera(self) -> bool:
        """Attempt to connect to the first available event camera.

        Returns True on success, False otherwise. Safe to call repeatedly.
        """
        self.log_message("Attempting to connect to event camera...")
        if self.camera is not None and self.slicer is not None:
            # Already connected
            if self._camera_thread is None or not self._camera_thread.is_alive():
                self._camera_thread = threading.Thread(target=self._camera_loop, daemon=True)
                self._camera_thread.start()
            self.log_message("Camera already connected; ensured camera loop is running.")
            return True

        try:
            cam = Camera.from_first_available()
        except Exception:
            self.camera = None
            self.slicer = None
            self.log_message("Camera connection failed: exception from Camera.from_first_available().")
            return False

        if cam is None:
            self.camera = None
            self.slicer = None
            self.log_message("Camera connection failed: no camera available.")
            return False

        self.camera = cam
        try:
            self.slicer = CameraStreamSlicer(self.camera.move())
        except Exception:
            self.camera = None
            self.slicer = None
            self.log_message("Camera connection failed: error creating stream slicer.")
            return False

        if self._camera_thread is None or not self._camera_thread.is_alive():
            self._camera_thread = threading.Thread(target=self._camera_loop, daemon=True)
            self._camera_thread.start()

        self.log_message("Camera connected successfully.")

        return True

    def connect_pi(self) -> bool:
        """Attempt to (re)connect the USB serial link to the peer Pi.

        Returns True on success, False otherwise. Safe to call repeatedly.
        """
        # If an open port already exists, consider it connected
        self.log_message(f"Attempting to connect to peer Pi on {self.serial_port}...")
        if self.ser is not None and getattr(self.ser, "is_open", False):
            if self._uart_thread is None or not self._uart_thread.is_alive():
                self._uart_thread = threading.Thread(target=self._uart_loop, daemon=True)
                self._uart_thread.start()
            self.log_message("Serial link already open; ensured UART listener is running.")
            return True

        # Try opening the serial port
        try:
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=0.5,
            )
        except Exception:
            self.ser = None
            self.log_message("Pi connection failed: unable to open serial port.")
            return False

        if self._uart_thread is None or not self._uart_thread.is_alive():
            self._uart_thread = threading.Thread(target=self._uart_loop, daemon=True)
            self._uart_thread.start()

        self.log_message("Pi connection established; UART listener started.")

        return True

    def _camera_loop(self):
        """Background thread: process event camera slices based on current mode.

        This loop runs in its own thread so that use_external_triggers and
        in_safe_state can change dynamically via UART while the camera is
        streaming.
        """
        if self.slicer is None:
            return

        try:
            for ev_slice in self.slicer:
                if self._stop_event.is_set():
                    break

                # Check current mode
                with self._state_lock:
                    use_trig = self.use_external_triggers
                    safe = self.in_safe_state

                # Always consume the slice to keep camera streaming, but only
                # drive GPIO when external triggers are enabled and not in a
                # safe state.
                evs = ev_slice.events
                if not (use_trig and not safe):
                    continue

                if evs.size <= 0:
                    continue

                evs_sorted_ts = evs["t"].sort()

                # Initialise window start on first event in packet
                if self._window_start_ts is None:
                    self._window_start_ts = evs_sorted_ts[0]
                    self._events_in_window = 0

                # If still within the current window, accumulate
                if (evs_sorted_ts[-1] - self._window_start_ts) <= self.time_window_us:
                    self._events_in_window += evs.size  # NOTE: This dismisses packets that lay in between windows in favour of processing speed
                else:
                    # Window completed: evaluate spike count and
                    # optionally drive a trigger pulse.
                    trig = self.trigger_logic(self._events_in_window)
                    if trig and self.frequency > 0:
                        period_s = 1.0 / float(self.frequency)
                        # Convert pulse width from µs to seconds
                        pw_s = max(float(self.pulse_width) / 1e6, 0.0)
                        off_s = max(period_s - pw_s, 0.0)
                        num_pulses = int(self.time_window_us / 1e6 / period_s)  # Fill window with pulses at f
                        self.trigger_out.blink(on_time=pw_s, off_time=off_s, n=num_pulses)

                    # Start a new window anchored at this event
                    self._window_start_ts = None
                    self._events_in_window = 0

        except Exception:
            # Treat any exception as a lost camera connection
            self.log_message("Camera stream error: disconnecting camera.")
            self.camera = None
            self.slicer = None

    def trigger_logic(self, events_in_window: int) -> bool:
        """Map spike count in a completed time window to trigger decision.

        events_in_window is the number of events observed during the most
        recent time window of length self.time_window_us (µs). This updates
        the internal frequency and returns True if a trigger pulse should be
        emitted for that window.
        """
        if events_in_window < self.low_spike_threshold:
            self.set_frequency(0)
            return False
        elif self.low_spike_threshold <= events_in_window < self.mid_spike_threshold:
            self.set_frequency(self.low_frequency)
            return True
        elif self.mid_spike_threshold <= events_in_window < self.high_spike_threshold:
            self.set_frequency(self.mid_frequency)
            return True
        else:
            self.set_frequency(self.high_frequency)
            return True


class RobotController:
    def __init__(self, connect: bool = True):
        self.base_frame = (353, 18, -35, 0, 0, 90)	# base frame: x->front, y->left, z->up, rz->anticlockwise
        self.work_frame = (353, 18, -80, 0, 0, 90)
        self.tap_move = None
        self.tcp = (0, 0, -50, 0, 0, 0)
        self.linear_speed = 0

        # In-memory metadata dictionary describing the robot configuration
        self.meta = {
            "base_frame": list(self.base_frame),
            "work_frame": list(self.work_frame),
            "tap_move": self.tap_move,
            "tcp": list(self.tcp),
            "linear_speed": self.linear_speed,
        }

        # Optional hardware connection. For UI meta operations we may
        # construct this controller with connect=False to avoid blocking
        # if robot hardware is unavailable.
        self.robot = None
        if connect:
            try:
                self.robot = self._make_robot()
            except Exception:
                self.robot = None

            # If the robot was created successfully, try to apply the
            # configured frames and speed, but do not discard the robot
            # object if any of these assignments fail.
            if self.robot is not None:
                try:
                    self.robot.tcp = self.tcp
                except Exception:
                    pass
                try:
                    self.robot.coord_frame = self.base_frame
                except Exception:
                    pass
                try:
                    self.robot.speed = self.linear_speed
                except Exception:
                    pass

    def _make_robot(self) -> AsyncRobot:
        return AsyncRobot(SyncRobot(Controller()))

    def close_robot(self):
        if self.robot is not None:
            try:
                self.robot.close()
            except Exception:
                pass
            self.robot = None

    def make_meta(self, filename):
        """Save the current robot configuration to a JSON meta file."""
        # Refresh meta from current attributes
        self.meta = {
            "base_frame": list(self.base_frame),
            "work_frame": list(self.work_frame),
            "tap_move": self.tap_move,
            "tcp": list(self.tcp),
            "linear_speed": self.linear_speed,
        }
        with open(filename, "w", encoding="utf-8") as f:
            json.dump(self.meta, f, indent=2)

    def get_meta(self, filename):
        """Load robot configuration from a JSON meta file into this instance."""
        with open(filename, "r", encoding="utf-8") as f:
            data = json.load(f)

        # Update attributes from the loaded meta, falling back to existing
        # values when keys are missing.
        if "base_frame" in data:
            self.base_frame = tuple(data["base_frame"])
        if "work_frame" in data:
            self.work_frame = tuple(data["work_frame"])
        if "tap_move" in data:
            self.tap_move = data["tap_move"]
        if "tcp" in data:
            self.tcp = tuple(data["tcp"])
        if "linear_speed" in data:
            self.linear_speed = data["linear_speed"]

        # Keep meta in sync
        self.meta = {
            "base_frame": list(self.base_frame),
            "work_frame": list(self.work_frame),
            "tap_move": self.tap_move,
            "tcp": list(self.tcp),
            "linear_speed": self.linear_speed,
        }

        # If a robot instance exists, update its runtime configuration
        if getattr(self, "robot", None) is not None:
            try:
                self.robot.tcp = self.tcp
            except Exception:
                pass
            try:
                self.robot.coord_frame = self.base_frame
            except Exception:
                pass
            try:
                self.robot.speed = self.linear_speed
            except Exception:
                pass



class CameraIOApp:
    """Simple Tk UI wrapper around IO2Pulse for manual control.

    - Top-right 'X' button triggers a confirmed shutdown.
    - Buttons to connect/reconnect the event camera and the peer Pi.
    """

    def __init__(self, master: tk.Tk):
        self.master = master
        self.master.title("Camera IO2Pulse Controller")

        # Underlying IO2Pulse controller
        self.io = IO2Pulse()
        
        # Underlying Robot controller (created on demand)
        self.robot_ctrl: RobotController | None = None
        self._robot_connecting = False
        self._robot_connect_timeout_ms = 5000

        # Shutdown button (top-right, denoted by 'X')
        self.shutdown_button = tk.Button(master, text="X", command=self.on_shutdown, width=3)
        self.shutdown_button.grid(row=0, column=2, padx=10, pady=10, sticky="ne")

        # Camera status and connect button
        self.camera_status = tk.Label(master, text="Camera: disconnected")
        self.camera_status.grid(row=0, column=0, padx=10, pady=10, sticky="w")

        self.camera_button = tk.Button(master, text="Connect Camera", command=self.on_connect_camera, width=18)
        self.camera_button.grid(row=1, column=0, padx=10, pady=5, sticky="w")

        # Pi (serial) status and connect/reconnect button
        self.pi_status = tk.Label(master, text="Pi link: disconnected")
        self.pi_status.grid(row=2, column=0, padx=10, pady=10, sticky="w")

        self.pi_button = tk.Button(master, text="Connect / Reconnect Pi", command=self.on_connect_pi, width=18)
        self.pi_button.grid(row=3, column=0, padx=10, pady=5, sticky="w")

        # Robot status and connect button (right side)
        self.robot_status = tk.Label(master, text="Robot: disconnected")
        self.robot_status.grid(row=1, column=2, padx=10, pady=10, sticky="w")

        self.robot_button = tk.Button(master, text="Connect Robot", command=self.on_connect_robot, width=18)
        self.robot_button.grid(row=2, column=2, padx=10, pady=5, sticky="w")

        # Robot meta load/save buttons (right side)
        self.load_meta_button = tk.Button(master, text="Load Meta", command=self.on_load_robot_meta, width=12)
        self.load_meta_button.grid(row=3, column=2, padx=10, pady=5, sticky="w")

        self.save_meta_button = tk.Button(master, text="Save Meta", command=self.on_save_robot_meta, width=12)
        self.save_meta_button.grid(row=4, column=2, padx=10, pady=5, sticky="w")

        # Display box for current robot meta data (right side)
        self.robot_meta_text = tk.Text(master, width=40, height=6, state="disabled")
        self.robot_meta_text.grid(row=5, column=2, rowspan=2, padx=10, pady=5, sticky="nw")

        # Editable fields for key robot meta parameters (labels left of entries)
        tk.Label(master, text="Base Frame").grid(row=7, column=2, padx=10, pady=(10, 2), sticky="e")
        self.base_frame_entry = tk.Entry(master, width=30)
        self.base_frame_entry.grid(row=7, column=3, padx=5, pady=2, sticky="w")

        tk.Label(master, text="Work Frame").grid(row=8, column=2, padx=10, pady=(10, 2), sticky="e")
        self.work_frame_entry = tk.Entry(master, width=30)
        self.work_frame_entry.grid(row=8, column=3, padx=5, pady=2, sticky="w")

        tk.Label(master, text="TCP").grid(row=9, column=2, padx=10, pady=(10, 2), sticky="e")
        self.tcp_entry = tk.Entry(master, width=30)
        self.tcp_entry.grid(row=9, column=3, padx=5, pady=2, sticky="w")

        tk.Label(master, text="Linear Speed").grid(row=10, column=2, padx=10, pady=(10, 2), sticky="e")
        self.linear_speed_entry = tk.Entry(master, width=30)
        self.linear_speed_entry.grid(row=10, column=3, padx=5, pady=2, sticky="w")

        tk.Label(master, text="Tap Move").grid(row=11, column=2, padx=10, pady=(10, 2), sticky="e")
        self.tap_move_entry = tk.Entry(master, width=30)
        self.tap_move_entry.grid(row=11, column=3, padx=5, pady=2, sticky="w")

        self.apply_meta_button = tk.Button(master, text="Apply Meta Changes", command=self.on_apply_robot_meta, width=18)
        self.apply_meta_button.grid(row=12, column=3, padx=10, pady=5, sticky="e")

        # State indicators for External Trigger and Safe State flags
        self.trigger_state_label = tk.Label(master, text="External Trigger: OFF")
        self.trigger_state_label.grid(row=6, column=0, padx=10, pady=5, sticky="w")

        self.safe_state_label = tk.Label(master, text="Safe State: INACTIVE")
        self.safe_state_label.grid(row=7, column=0, padx=10, pady=5, sticky="w")

        # Indicators for pulse width and stimulation amplitude driven by UART
        self.pulse_width_var = tk.StringVar(value="Pulse Width: —")
        self.pulse_width_label = tk.Label(master, textvariable=self.pulse_width_var)
        self.pulse_width_label.grid(row=8, column=0, padx=10, pady=5, sticky="w")

        self.stim_amp_var = tk.StringVar(value="Stim Amplitude: —")
        self.stim_amp_label = tk.Label(master, textvariable=self.stim_amp_var)
        self.stim_amp_label.grid(row=9, column=0, padx=10, pady=5, sticky="w")

        # Indicator for current trigger frequency as computed from events
        self.freq_var = tk.StringVar(value="Frequency: —")
        self.freq_label = tk.Label(master, textvariable=self.freq_var)
        self.freq_label.grid(row=10, column=0, padx=10, pady=5, sticky="w")

        # Log window for connection attempts and messages from the other Pi
        self.log_text = tk.Text(master, width=60, height=10, state="disabled")
        self.log_text.grid(row=13, column=0, columnspan=4, padx=10, pady=10, sticky="nsew")

        # Periodic polling of IO2Pulse log queue and state flags
        self._poll_logs()
        self._update_state_labels()
        self._update_robot_meta_box()

        # Ensure window close via titlebar also performs a safe shutdown
        self.master.protocol("WM_DELETE_WINDOW", self.on_shutdown)

    # UI callbacks
    def on_shutdown(self):
        """Confirm shutdown, then stop IO2Pulse and close the UI."""
        if not messagebox.askokcancel("Shutdown", "Shut down IO2Pulse and close the window?"):
            return
        # Cleanly close any active robot connection first
        try:
            if self.robot_ctrl is not None:
                self.robot_ctrl.close_robot()
        except Exception:
            pass
        try:
            self.io.stop()
        except Exception:
            pass
        self.master.destroy()

    def on_connect_camera(self):
        """Attempt to connect (or reconnect) the event camera."""
        ok = False
        try:
            ok = self.io.connect_camera()
        except Exception:
            ok = False

        if ok:
            self.camera_status.config(text="Camera: connected")
            self._append_log("Camera connected.")
            try:
                self.camera_button.config(state="disabled")
            except Exception:
                pass
        else:
            self.camera_status.config(text="Camera: disconnected")
            messagebox.showerror("Camera Connection", "Unable to connect to an event camera.")
            self._append_log("Camera connection failed.")
            try:
                self.camera_button.config(state="normal")
            except Exception:
                pass

    def on_connect_pi(self):
        """Attempt to connect (or reconnect) the USB link to the peer Pi."""
        ok = False
        try:
            ok = self.io.connect_pi()
        except Exception:
            ok = False

        if ok:
            self.pi_status.config(text=f"Pi link: connected on {self.io.serial_port}")
            self._append_log(f"Pi connected on {self.io.serial_port}.")
            try:
                self.pi_button.config(state="disabled")
            except Exception:
                pass
        else:
            self.pi_status.config(text="Pi link: disconnected")
            messagebox.showerror("Pi Connection", f"Unable to open serial port {self.io.serial_port}.")
            self._append_log("Pi connection failed.")
            try:
                self.pi_button.config(state="normal")
            except Exception:
                pass

    def on_connect_robot(self):
        """Attempt to construct and connect the Robot without blocking the UI.

        Spawns a background thread to create RobotController and uses a
        soft timeout so the UI can recover if the connection takes too long.
        """
        # If we already have a controller and robot object, do nothing
        if self.robot_ctrl is not None and getattr(self.robot_ctrl, "robot", None) is not None:
            return
        if self._robot_connecting:
            return

        self._robot_connecting = True
        self.robot_status.config(text="Robot: connecting...")
        try:
            self.robot_button.config(state="disabled")
        except Exception:
            pass
        self._append_log("Starting robot connection attempt...")

        # Start connection in a worker thread to avoid blocking Tk
        def worker(start_time: float):
            ctrl = None
            err = None
            try:
                ctrl = RobotController()
            except Exception as e:  # noqa: BLE001
                err = e

            # Hand result back to Tk main loop
            try:
                self.master.after(0, lambda: self._on_robot_connect_result(start_time, ctrl, err))
            except Exception:
                # If we cannot schedule back to Tk, just drop the result
                pass

        start = time.monotonic()
        threading.Thread(target=worker, args=(start,), daemon=True).start()

        # Schedule a timeout check
        try:
            self.master.after(self._robot_connect_timeout_ms, self._check_robot_connect_timeout, start)
        except Exception:
            pass

    def _on_robot_connect_result(self, start_time: float, ctrl: RobotController | None, err: Exception | None):
        """Handle completion of the robot connection attempt on the Tk thread."""
        # If a newer attempt has started or we've already timed out, ignore
        if not self._robot_connecting:
            return

        elapsed_ms = int((time.monotonic() - start_time) * 1000)

        if err is not None:
            self.robot_ctrl = None
            self.robot_status.config(text="Robot: disconnected")
            self._append_log(f"Robot connection failed: {err}")
            self._robot_connecting = False
            try:
                self.robot_button.config(state="normal")
            except Exception:
                pass
            return

        self.robot_ctrl = ctrl
        robot_obj = getattr(self.robot_ctrl, "robot", None) if self.robot_ctrl is not None else None

        if robot_obj is not None and elapsed_ms <= self._robot_connect_timeout_ms:
            self.robot_status.config(text="Robot: connected")
            self._append_log("Robot connected successfully.")
            self._robot_connecting = False
            try:
                self.robot_button.config(state="disabled")
            except Exception:
                pass
        else:
            # Either no robot object or connection exceeded timeout
            self.robot_ctrl = None
            self.robot_status.config(text="Robot: disconnected")
            if elapsed_ms > self._robot_connect_timeout_ms:
                self._append_log("Robot connection timed out.")
            else:
                self._append_log("Robot connection failed.")
            self._robot_connecting = False
            try:
                self.robot_button.config(state="normal")
            except Exception:
                pass

    def _check_robot_connect_timeout(self, start_time: float):
        """Soft timeout: if still connecting after the window, re-enable UI."""
        if not self._robot_connecting:
            return
        # If another attempt has started since, don't interfere
        # (we only care about cases where the worker is still blocking).
        self._append_log("Robot connection taking too long; marking as failed.")
        self.robot_ctrl = None
        self.robot_status.config(text="Robot: disconnected")
        self._robot_connecting = False
        try:
            self.robot_button.config(state="normal")
        except Exception:
            pass

    def _require_robot_ctrl(self) -> bool:
        """Ensure a RobotController exists; used for meta load/save.

        Returns True if available, False otherwise. Does not attempt a robot
        hardware connection if one is not already present.
        """
        if self.robot_ctrl is None:
            # Create a controller shell without forcing the UI to hang if
            # hardware is unreachable; rely on its own error handling.
            try:
                # Do not attempt a hardware connection here; this is only
                # for working with meta data and should not block the UI.
                self.robot_ctrl = RobotController(connect=False)
            except Exception as e:  # noqa: BLE001
                self.robot_ctrl = None
                self._append_log(f"Failed to create RobotController: {e}")
                messagebox.showerror("Robot Meta", "Unable to create robot controller for meta operations.")
                return False
        # Update meta display with whatever default meta we have
        self._update_robot_meta_box()
        return True

    def on_load_robot_meta(self):
        """Let the user choose a meta.json file and load it into the robot controller."""
        if not self._require_robot_ctrl():
            return

        filename = filedialog.askopenfilename(
            title="Select Robot Meta File",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
        )
        if not filename:
            return

        try:
            self.robot_ctrl.get_meta(filename)
        except Exception as e:  # noqa: BLE001
            self._append_log(f"Failed to load robot meta from {filename}: {e}")
            messagebox.showerror("Robot Meta", f"Failed to load meta file:\n{e}")
            return

        self._append_log(f"Loaded robot meta from {filename}.")
        try:
            self.robot_status.config(text="Robot: meta loaded")
        except Exception:
            pass
        self._update_robot_meta_box()

    def on_save_robot_meta(self):
        """Save the current robot meta dictionary to a user-selected file."""
        if not self._require_robot_ctrl():
            return

        filename = filedialog.asksaveasfilename(
            title="Save Robot Meta File",
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
        )
        if not filename:
            return

        try:
            self.robot_ctrl.make_meta(filename)
        except Exception as e:  # noqa: BLE001
            self._append_log(f"Failed to save robot meta to {filename}: {e}")
            messagebox.showerror("Robot Meta", f"Failed to save meta file:\n{e}")
            return

        self._append_log(f"Saved robot meta to {filename}.")
        self._update_robot_meta_box()

    def on_apply_robot_meta(self):
        """Apply values from the editable meta fields into the robot meta.

        Parses JSON for the frame- and TCP-like fields and a float for
        linear_speed. Updates RobotController attributes and meta dict,
        and pushes changes to any active robot instance.
        """
        if not self._require_robot_ctrl():
            return

        ctrl = self.robot_ctrl
        if ctrl is None:
            return

        # Helper to parse JSON from an entry, returning None on failure
        def _parse_json_entry(entry_widget):
            text = entry_widget.get().strip()
            if not text:
                return None
            try:
                return json.loads(text)
            except Exception:
                return None

        # Start from current meta
        meta = dict(getattr(ctrl, "meta", {}))

        base_val = _parse_json_entry(self.base_frame_entry)
        if isinstance(base_val, (list, tuple)) and len(base_val) == 6:
            ctrl.base_frame = tuple(base_val)
            meta["base_frame"] = list(ctrl.base_frame)

        work_val = _parse_json_entry(self.work_frame_entry)
        if isinstance(work_val, (list, tuple)) and len(work_val) == 6:
            ctrl.work_frame = tuple(work_val)
            meta["work_frame"] = list(ctrl.work_frame)

        tcp_val = _parse_json_entry(self.tcp_entry)
        if isinstance(tcp_val, (list, tuple)) and len(tcp_val) == 6:
            ctrl.tcp = tuple(tcp_val)
            meta["tcp"] = list(ctrl.tcp)

        lin_text = self.linear_speed_entry.get().strip()
        if lin_text:
            try:
                ctrl.linear_speed = float(lin_text)
                meta["linear_speed"] = ctrl.linear_speed
            except Exception:
                pass

        tap_val = _parse_json_entry(self.tap_move_entry)
        # tap_move can be any JSON-serialisable structure or None
        if tap_val is not None or self.tap_move_entry.get().strip() == "":
            ctrl.tap_move = tap_val
            meta["tap_move"] = ctrl.tap_move

        # Store updated meta back on controller
        ctrl.meta = meta

        # If robot hardware is connected, push configuration
        robot_obj = getattr(ctrl, "robot", None)
        if robot_obj is not None:
            try:
                robot_obj.tcp = ctrl.tcp
            except Exception:
                pass
            try:
                robot_obj.coord_frame = ctrl.base_frame
            except Exception:
                pass
            try:
                robot_obj.speed = ctrl.linear_speed
            except Exception:
                pass

        self._append_log("Applied robot meta changes from UI fields.")
        self._update_robot_meta_box()

    # Internal helpers
    def _append_log(self, message: str):
        """Append a line to the log window from the Tk main thread."""
        try:
            ts = time.strftime("%H:%M:%S")
        except Exception:
            ts = "--:--:--"
        line = f"[{ts}] {message}\n"
        try:
            self.log_text.configure(state="normal")
            self.log_text.insert("end", line)
            self.log_text.see("end")
            self.log_text.configure(state="disabled")
        except Exception:
            pass

    def _update_robot_meta_box(self):
        """Refresh the on-screen view of the current robot meta data."""
        text = ""
        try:
            if self.robot_ctrl is not None and getattr(self.robot_ctrl, "meta", None) is not None:
                try:
                    text = json.dumps(self.robot_ctrl.meta, indent=2)
                except Exception:
                    text = str(self.robot_ctrl.meta)
        except Exception:
            text = ""

        try:
            self.robot_meta_text.configure(state="normal")
            self.robot_meta_text.delete("1.0", "end")
            if text:
                self.robot_meta_text.insert("end", text)
            self.robot_meta_text.configure(state="disabled")
        except Exception:
            pass

        # Also keep the editable fields in sync with the current meta
        try:
            ctrl = self.robot_ctrl
            meta = getattr(ctrl, "meta", {}) if ctrl is not None else {}

            def _set_entry(entry_widget, value, as_json: bool = False):
                try:
                    entry_widget.delete(0, "end")
                    if value is None:
                        return
                    if as_json:
                        entry_widget.insert(0, json.dumps(value))
                    else:
                        entry_widget.insert(0, str(value))
                except Exception:
                    pass

            _set_entry(self.base_frame_entry, meta.get("base_frame"), as_json=True)
            _set_entry(self.work_frame_entry, meta.get("work_frame"), as_json=True)
            _set_entry(self.tcp_entry, meta.get("tcp"), as_json=True)
            _set_entry(self.linear_speed_entry, meta.get("linear_speed"), as_json=False)
            _set_entry(self.tap_move_entry, meta.get("tap_move"), as_json=True)
        except Exception:
            pass

    def _poll_logs(self):
        """Periodically drain IO2Pulse log messages and display them."""
        try:
            msgs = self.io.get_log_messages(max_items=50)
        except Exception:
            msgs = []
        for msg in msgs:
            self._append_log(msg)
        # Schedule next poll
        try:
            self.master.after(200, self._poll_logs)
        except Exception:
            pass

    def _update_state_labels(self):
        """Refresh the External Trigger and Safe State indicators."""
        try:
            use_trig = self.io.using_external_triggers()
        except Exception:
            use_trig = False
        try:
            safe = self.io.is_safe()
        except Exception:
            safe = False

        trig_text = "External Trigger: ON" if use_trig else "External Trigger: OFF"
        safe_text = "Safe State: ACTIVE" if safe else "Safe State: INACTIVE"

        # Also reflect pulse width and stimulation amplitude values
        try:
            pw = self.io.get_pulse_width()
        except Exception:
            pw = None
        try:
            sa = self.io.get_stim_amplitude()
        except Exception:
            sa = None

        try:
            freq = self.io.get_frequency()
        except Exception:
            freq = None

        # Connection states for enabling/disabling connect buttons
        try:
            cam_connected = self.io.is_camera_connected()
        except Exception:
            cam_connected = False
        try:
            pi_connected = self.io.is_pi_connected()
        except Exception:
            pi_connected = False

        try:
            self.trigger_state_label.config(text=trig_text)
            self.safe_state_label.config(text=safe_text)
            if pw is not None:
                self.pulse_width_var.set(f"Pulse Width: {pw}")
            else:
                self.pulse_width_var.set("Pulse Width: —")
            if sa is not None:
                self.stim_amp_var.set(f"Stim Amplitude: {sa:.3f}")
            else:
                self.stim_amp_var.set("Stim Amplitude: —")
            if freq is not None and freq > 0:
                self.freq_var.set(f"Frequency: {freq} Hz")
            else:
                self.freq_var.set("Frequency: —")

            # Enable/disable connect buttons based on connection state
            self.camera_button.config(state="disabled" if cam_connected else "normal")
            self.pi_button.config(state="disabled" if pi_connected else "normal")
        except Exception:
            pass

        try:
            self.master.after(200, self._update_state_labels)
        except Exception:
            pass


def main():
    root = tk.Tk()
    app = CameraIOApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()