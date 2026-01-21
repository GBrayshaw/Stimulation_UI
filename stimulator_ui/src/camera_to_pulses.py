import tkinter as tk
from tkinter import messagebox

from metavision_sdk_core import PolarityFilterAlgorithm
from metavision_sdk_stream import Camera, CameraStreamSlicer
import numpy as np
from collections import deque
import serial
import threading
import time
import queue
from gpiozero import DigitalOutputDevice, DigitalInputDevice
import uart_protocol

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
            start = self.ser.read(1)
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

    def is_safe(self) -> bool:
        with self._state_lock:
            return self.in_safe_state

    def using_external_triggers(self) -> bool:
        with self._state_lock:
            return self.use_external_triggers

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

        for ev_slice in self.slicer:
            if self._stop_event.is_set():
                break

            _ = ev_slice.events.size    # Grab the number of events in this slice

            # Check current mode    
            with self._state_lock:
                use_trig = self.use_external_triggers
                safe = self.in_safe_state

            # Always consume the slice to keep camera streaming, but only
            # drive GPIO when external triggers are enabled and not in a
            # safe state.
            if use_trig and not safe:
                # TODO: Implement trigger output logic based on events
                if self.trigger_logic():
                    self.trigger_out.blink(on_time=0.01, off_time=0.99, n=1)  # Example: blink every second
            
    def trigger_logic(self) -> bool:
        """Define the logic to determine when to trigger output.

        This is a placeholder function and should be implemented based on
        specific requirements for triggering based on event data.
        """
        return True


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

        # State indicators for External Trigger and Safe State flags
        self.trigger_state_label = tk.Label(master, text="External Trigger: OFF")
        self.trigger_state_label.grid(row=4, column=0, padx=10, pady=5, sticky="w")

        self.safe_state_label = tk.Label(master, text="Safe State: INACTIVE")
        self.safe_state_label.grid(row=5, column=0, padx=10, pady=5, sticky="w")

        # Log window for connection attempts and messages from the other Pi
        self.log_text = tk.Text(master, width=60, height=12, state="disabled")
        self.log_text.grid(row=6, column=0, columnspan=3, padx=10, pady=10, sticky="nsew")

        # Periodic polling of IO2Pulse log queue and state flags
        self._poll_logs()
        self._update_state_labels()

        # Ensure window close via titlebar also performs a safe shutdown
        self.master.protocol("WM_DELETE_WINDOW", self.on_shutdown)

    # UI callbacks
    def on_shutdown(self):
        """Confirm shutdown, then stop IO2Pulse and close the UI."""
        if not messagebox.askokcancel("Shutdown", "Shut down IO2Pulse and close the window?"):
            return
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
        else:
            self.camera_status.config(text="Camera: disconnected")
            messagebox.showerror("Camera Connection", "Unable to connect to an event camera.")
            self._append_log("Camera connection failed.")

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
        else:
            self.pi_status.config(text="Pi link: disconnected")
            messagebox.showerror("Pi Connection", f"Unable to open serial port {self.io.serial_port}.")
            self._append_log("Pi connection failed.")

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

        try:
            self.trigger_state_label.config(text=trig_text)
            self.safe_state_label.config(text=safe_text)
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