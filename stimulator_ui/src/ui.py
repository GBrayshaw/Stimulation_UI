from tkinter import Tk, Button, Checkbutton, IntVar, Label, StringVar, Text, Entry, END, DISABLED, NORMAL, messagebox, Frame, Radiobutton, Scrollbar, Canvas
from datetime import datetime
from stim_io import UART_COMMS
import uart_protocol
from user_io import USER_COMMS
import serial
import serial.tools.list_ports
import os
import subprocess
import time

class AppUI:
    def __init__(self, master):
        self.master = master
        master.title("Test Stimulator App") 

        # UI Display variables
        self.stim_amplitude = 0.00
        self.pulse_width = 0.00
        self.stim_frequency = 0.00
        self.burst_length = 0.00
        self.pending_stim_amplitude = 0.00
        self.pending_pulse_width = 0.00
        self.pending_stim_frequency = 0.00
        self.pending_burst_length = 0.00
        self.nerve_impedance = 0.00
        # UI limits
        self._max_amp_ua = 10.0
        # Runtime state flags for control logic
        self.interlocks_on = None  # True when engaged; False when disengaged; None unknown
        self.stim_on = None        # True when stimulation active; False when inactive; None unknown
        # Last logged states so we only log when something actually changes
        self._last_interlocks_on = None
        self._last_stim_on = None
        self._last_status_code = None

        # Display Labels
        self.stim_amplitude_var = StringVar()
        # Show placeholder until first value received from control board
        self.stim_amplitude_var.set("Stim Amplitude: — uA")
        self.stim_amplitude_label = Label(master, textvariable=self.stim_amplitude_var)
        self.stim_amplitude_label.grid(row=0, column=0, columnspan=2, padx=10, pady=10)

        self.pulse_width_var = StringVar()
        # Show placeholder until first value received from control board
        self.pulse_width_var.set("Pulse Width: —")
        self.pulse_width_label = Label(master, textvariable=self.pulse_width_var)
        self.pulse_width_label.grid(row=2, column=0, columnspan=2, padx=10, pady=10)

        # self.nerve_impedance_var = StringVar()
        # self.nerve_impedance_var.set(f"Nerve Impedance: {self.nerve_impedance:.2f}ohms")
        # self.nerve_impedance_label = Label(master, textvariable=self.nerve_impedance_var)
        # self.nerve_impedance_label.grid(row=2, column=2, padx=10, pady=10)

        # UI Buttons for amplitude and pulse width
        self.stim_up_but = Button(master, text="+", command=self.stim_amp_up, width=10, height=2, state=DISABLED)
        self.stim_up_but.grid(row=1, column=0, padx=10, pady=10)

        self.stim_down_but = Button(master, text="-", command=self.stim_amp_down, width=10, height=2, state=DISABLED)
        self.stim_down_but.grid(row=1, column=1, padx=10, pady=10)

        self.pulse_up_but = Button(master, text="+", command=self.pulse_width_up, width=10, height=2, state=DISABLED)
        self.pulse_up_but.grid(row=3, column=0, padx=10, pady=10)

        self.pulse_down_but = Button(master, text="-", command=self.pulse_width_down, width=10, height=2, state=DISABLED)
        self.pulse_down_but.grid(row=3, column=1, padx=10, pady=10)

        # Parameter entry boxes (left side)
        self.stim_amplitude_entry = Entry(master, state=DISABLED)
        self.stim_amplitude_entry.grid(row=1, column=2, padx=10, pady=10)
        self.stim_amplitude_entry.bind("<Return>", self.update_stim_amplitude)

        self.pulse_width_entry = Entry(master, state=DISABLED)
        self.pulse_width_entry.grid(row=3, column=2, padx=10, pady=10)
        self.pulse_width_entry.bind("<Return>", self.update_pulse_width)

        # Additional stimulation parameters: frequency and burst length
        self.stim_freq_label = Label(master, text="Stim Frequency (Hz)")
        # Align header with Stim Amplitude / Pulse Width headers
        self.stim_freq_label.grid(row=4, column=0, columnspan=2, padx=10, pady=(10, 0))
        self.stim_freq_entry = Entry(master, state=DISABLED)
        # Align box with Stim Amplitude / Pulse Width boxes (entry column)
        self.stim_freq_entry.grid(row=5, column=2, padx=10, pady=5)
        # Up/down controls for stimulation frequency
        self.stim_freq_up_but = Button(master, text="+", command=self.stim_freq_up, width=10, height=2, state=DISABLED)
        self.stim_freq_up_but.grid(row=5, column=0, padx=10, pady=5)
        self.stim_freq_down_but = Button(master, text="-", command=self.stim_freq_down, width=10, height=2, state=DISABLED)
        self.stim_freq_down_but.grid(row=5, column=1, padx=10, pady=5)
        self.stim_freq_entry.bind("<Return>", self.update_stim_freq)

        self.burst_len_label = Label(master, text="Burst Length (us)")
        # Align header with Stim Amplitude / Pulse Width headers
        self.burst_len_label.grid(row=6, column=0, columnspan=2, padx=10, pady=(10, 0))
        self.burst_len_entry = Entry(master, state=DISABLED)
        # Align box with Stim Amplitude / Pulse Width boxes (entry column)
        self.burst_len_entry.grid(row=7, column=2, padx=10, pady=5)
        # Up/down controls for burst length
        self.burst_len_up_but = Button(master, text="+", command=self.burst_len_up, width=10, height=2, state=DISABLED)
        self.burst_len_up_but.grid(row=7, column=0, padx=10, pady=5)
        self.burst_len_down_but = Button(master, text="-", command=self.burst_len_down, width=10, height=2, state=DISABLED)
        self.burst_len_down_but.grid(row=7, column=1, padx=10, pady=5)
        self.burst_len_entry.bind("<Return>", self.update_burst_len)

        # Parameter actions below all parameter boxes
        self.done_but = Button(master, text="Set Parameters", command=self.apply_settings, width=12, height=2)
        self.done_but.grid(row=8, column=0, padx=10, pady=10, sticky="e")

        self.poll_status_but = Button(master, text="Get Parameters", command=self.poll_status, width=12, height=2, state=DISABLED)
        self.poll_status_but.grid(row=8, column=1, padx=10, pady=10, sticky="w")

        # Right-hand control panel: mode, actions, switches, and debug toggle
        self.right_panel = Frame(master)
        self.right_panel.grid(row=0, column=3, rowspan=12, padx=10, pady=10, sticky="n")

        # Mode selection: Recording vs Stimulation
        self.mode_var = IntVar()
        self.mode_var.set(0)  # 0 = Recording, 1 = Stimulation
        self.mode_label = Label(self.right_panel, text="Mode")
        self.mode_label.pack(anchor="center", padx=5, pady=(0, 2))
        self.mode_record_radio = Radiobutton(self.right_panel, text="Recording", variable=self.mode_var, value=0,
                 command=self.toggle_recording, state=DISABLED)
        self.mode_record_radio.pack(anchor="center", padx=10, pady=1)
        self.mode_stim_radio = Radiobutton(self.right_panel, text="Stimulation", variable=self.mode_var, value=1,
               command=self.toggle_recording, state=DISABLED)
        self.mode_stim_radio.pack(anchor="center", padx=10, pady=1)

        # Trigger source selection (Internal vs External), under Mode
        self.trigger_mode_var = IntVar()
        self.trigger_mode_var.set(0)  # 0 = Internal, 1 = External
        self.triggers_label = Label(self.right_panel, text="Triggers")
        self.triggers_label.pack(anchor="center", padx=5, pady=(8, 0))
        self.trigger_internal_radio = Radiobutton(self.right_panel, text="Internal", variable=self.trigger_mode_var, value=0,
                  command=self.toggle_trigger, state=DISABLED)
        self.trigger_internal_radio.pack(anchor="center", padx=10, pady=1)
        self.trigger_external_radio = Radiobutton(self.right_panel, text="External", variable=self.trigger_mode_var, value=1,
                  command=self.toggle_trigger, state=DISABLED)
        self.trigger_external_radio.pack(anchor="center", padx=10, pady=1)

        # Control actions stacked vertically
        self.start_but = Button(self.right_panel, text="Unlock Interlocks", command=self.start_system,
                width=16, height=2, state=DISABLED)
        self.start_but.pack(padx=5, pady=(12, 4))
        self.start_stim_but = Button(self.right_panel, text="Start Stim", command=self.start_stim,
                 width=12, height=2, state=DISABLED)
        self.start_stim_but.pack(padx=5, pady=4)
        self.STOP_but = Button(self.right_panel, text="STOP", command=self.STOP,
                   width=10, height=2, state=DISABLED)
        self.STOP_but.pack(padx=5, pady=4)
        self.reconnect_but = Button(self.right_panel, text="Reconnect", command=self.start_auto_connect, state=NORMAL)
        self.reconnect_but.pack(padx=5, pady=(4, 10))

        # PC vs User control selection, on right-hand side
        self.pc_user_mode_var = IntVar()
        self.pc_user_mode_var.set(0)  # 0 = User, 1 = PC
        self.pc_user_label = Label(self.right_panel, text="Control")
        self.pc_user_label.pack(anchor="center", padx=5, pady=(8, 0))
        self.pc_mode_user_radio = Radiobutton(self.right_panel, text="Manual", variable=self.pc_user_mode_var, value=0,
                      command=self.toggle_pc_user, state=DISABLED)
        self.pc_mode_user_radio.pack(anchor="center", padx=10, pady=1)
        self.pc_mode_pc_radio = Radiobutton(self.right_panel, text="Tablet", variable=self.pc_user_mode_var, value=1,
                    command=self.toggle_pc_user, state=DISABLED)
        self.pc_mode_pc_radio.pack(anchor="center", padx=10, pady=1)

        # Show/hide debug log toggle and bottom log bar
        self.show_debug_var = IntVar()
        self.show_debug_var.set(1)
        # Configure columns to allow bottom bar to stretch
        try:
            for c in range(0, 5):
                self.master.grid_columnconfigure(c, weight=1)
        except Exception:
            pass
        # Bottom log frame spanning full width
        self.log_frame = Frame(master)
        self.log_frame.grid(row=15, column=0, columnspan=5, padx=10, pady=(0, 10), sticky="ew")
        try:
            self.log_frame.grid_columnconfigure(0, weight=1)
        except Exception:
            pass
        # Event log as a thin, scrollable bar
        self.event_log = Text(self.log_frame, height=4, wrap="word")
        self.event_log.grid(row=0, column=0, padx=(0, 4), sticky="ew")
        self.log_scroll = Scrollbar(self.log_frame, orient="vertical", command=self.event_log.yview)
        self.log_scroll.grid(row=0, column=1, sticky="ns")
        try:
            self.event_log.configure(yscrollcommand=self.log_scroll.set)
        except Exception:
            pass
        # Show Debug toggle aligned to the right of the bar
        self.show_debug_check = Checkbutton(self.log_frame, text="Show Debug",
                    variable=self.show_debug_var,
                    command=self.toggle_debug)
        self.show_debug_check.grid(row=0, column=2, padx=(8, 0), sticky="e")

        # Shutdown button (upper-right)
        self.shutdown_but = Button(master, text="Shutdown", command=self.shutdown_system, width=12, height=2)
        self.shutdown_but.grid(row=0, column=4, padx=10, pady=10)

        # Right-side plot area (replaces former right-side log)
        self.plot_frame = Frame(master)
        self.plot_frame.grid(row=1, column=4, rowspan=13, padx=10, pady=10, sticky="nsew")
        try:
            self.plot_frame.grid_rowconfigure(0, weight=1)
            self.plot_frame.grid_columnconfigure(0, weight=1)
        except Exception:
            pass
        self.plot_canvas = Canvas(self.plot_frame, background="#ffffff", height=300, width=420, highlightthickness=1, highlightbackground="#cccccc")
        self.plot_canvas.grid(row=0, column=0, sticky="nsew")

        # Initialize serial communication
        # We no longer use saved serial numbers. Control board is on a fixed UART port,
        # the user board is expected to be the only connected USB serial device.
        self.control_board_serial = "/dev/ttyAMA0"
        self.user_board_serial = None
        self.control_uart = None
        self.user_board = None

        # USB serial link to peer UI (other Raspberry Pi)
        # Default device path may need to be adjusted per system.
        self.peer_ui_serial = "/dev/ttyUSB0"
        self.peer_ui_baudrate = getattr(uart_protocol, "CONTROL_BOARD_UART_BAUD_RATE", 9600)
        self.peer_ui_ser = None
        self._peer_ui_connected = False

        self.pc_usr_toggle = 1
        # Heartbeat monitoring
        self._hb_timeout_ms = getattr(uart_protocol, 'HEARTBEAT_TIMEOUT_MS', 300)
        # Relax watchdog: use a larger multiplier to tolerate longer gaps
        # between incoming packets from the control board, especially during
        # initial power-on while the stim board is still syncing.
        self._hb_timeout_multiplier = 5.0
        self._hb_check_interval_ms = max(50, int(self._hb_timeout_ms / 2))
        self._hb_monitor_id = None
        self._hb_warmup_until = None  # grace window after connect
        self._hb_tripped = False

        # Connection attempt state (non-blocking auto-connect)
        self._connect_after_id = None
        self._connect_retry_ms = 250
        self._connect_attempts = 0

        # Periodic mode resend (redundant STIM/RECORD signalling to control board)
        self._mode_resend_interval_ms = 2000
        self._mode_resend_id = None

        # Start UI immediately and begin seeking the control board
        self.log_event(f"Seeking control board on {self.control_board_serial}...")
        self.start_auto_connect()
        # self.initialise_user_board()
        # Initial plot render
        try:
            self.update_plot()
        except Exception:
            pass

    def connect_control_board(self):
        # Backward-compatible entrypoint: kick off non-blocking auto-connect
        self.start_auto_connect()

    def start_auto_connect(self):
        # Cancel any pending attempts to avoid duplicates
        if self._connect_after_id is not None:
            try:
                self.master.after_cancel(self._connect_after_id)
            except Exception:
                pass
            self._connect_after_id = None
        self._connect_attempts = 0
        # Disable reconnect while attempting/connected
        try:
            self.reconnect_but.config(state=DISABLED)
        except Exception:
            pass
        self._connect_after_id = self.master.after(10, self._connect_step)

    def _connect_step(self):
        self._connect_after_id = None
        if not self.control_board_serial:
            self.log_event("Control board serial port not set. Please edit the code to set the port.")
            self.disable_uart_controls()
            return
        try:
            if self.control_uart is None:
                self.control_uart = UART_COMMS(port=self.control_board_serial, baudrate=9600)
            ok = False
            try:
                # Keep handshake short to avoid blocking the UI loop
                ok = self.control_uart.handshake(overall_timeout_s=0.5, resend_interval_s=0.25)
            except Exception as e:
                ok = False
            if ok:
                self.log_event(f"Control board connected on port {self.control_board_serial}.")
                self.enable_uart_controls()
                # Start heartbeat monitor
                self._hb_tripped = False
                # Start heartbeat monitor with an extended warmup to avoid
                # flagging the link as dead while the stim board is still
                # completing its own startup/READY sequence.
                self._hb_warmup_until = time.monotonic() + 8.0
                self._schedule_heartbeat_monitor()

                # After the control-board link is up, attempt to connect to
                # the peer UI over the USB serial link using a simple
                # ACK-based handshake defined by uart_protocol.
                try:
                    self.connect_peer_ui()
                except Exception:
                    # Any errors are logged inside connect_peer_ui; continue.
                    pass
                return
            else:
                # Not connected yet; retry
                self._connect_attempts += 1
                if self._connect_attempts % 10 == 0:
                    self.log_event("Still seeking control board...")
        except Exception as e:
            # On failure, clean up and retry later
            try:
                if self.control_uart:
                    self.control_uart.close()
            except Exception:
                pass
            self.control_uart = None
        # Schedule next attempt
        self._connect_after_id = self.master.after(self._connect_retry_ms, self._connect_step)

    def control_handshake(self):
        """Perform a handshake with the control board over UART using protocol ACK."""
        if not self.control_uart:
            self.log_event("No control board currently connected for handshake.")
            return False
        try:
            ok = self.control_uart.handshake()
            return bool(ok)
        except Exception as e:
            self.log_event(f"Handshake exception: {e}")
            return False

    def _schedule_heartbeat_monitor(self):
        # Schedule periodic heartbeat checks on the Tk loop
        if self._hb_monitor_id is not None:
            try:
                self.master.after_cancel(self._hb_monitor_id)
            except Exception:
                pass
        self._hb_monitor_id = self.master.after(self._hb_check_interval_ms, self._heartbeat_watchdog)

        # Also start event poller for incoming UART events
        try:
            if getattr(self, '_evt_poll_id', None) is not None:
                try:
                    self.master.after_cancel(self._evt_poll_id)
                except Exception:
                    pass
                self._evt_poll_id = None
            self._evt_poll_id = self.master.after(100, self._event_poller)
        except Exception:
            pass

        # Start periodic mode resend loop
        try:
            if getattr(self, '_mode_resend_id', None) is not None:
                try:
                    self.master.after_cancel(self._mode_resend_id)
                except Exception:
                    pass
                self._mode_resend_id = None
            self._mode_resend_id = self.master.after(self._mode_resend_interval_ms, self._mode_resend_loop)
        except Exception:
            pass

    def _mode_resend_loop(self):
        # Periodically re-send current mode (RECORD/STIM) to control board for redundancy
        self._mode_resend_id = self.master.after(self._mode_resend_interval_ms, self._mode_resend_loop)
        if not self.control_uart:
            return
        try:
            mode_val = self.mode_var.get()
        except Exception:
            return
        try:
            if mode_val == 0:
                # Recording mode
                self.control_uart.set_mode_record()
            else:
                # Stimulation mode
                self.control_uart.set_mode_stim()
        except Exception:
            pass

    def _heartbeat_watchdog(self):
        # Reschedule first to keep periodic cadence
        self._hb_monitor_id = self.master.after(self._hb_check_interval_ms, self._heartbeat_watchdog)
        if not self.control_uart:
            return
        # Warmup grace period after connect to avoid false positives
        try:
            if self._hb_warmup_until is not None and time.monotonic() < self._hb_warmup_until:
                return
        except Exception:
            pass
        try:
            threshold = int(self._hb_timeout_ms * self._hb_timeout_multiplier)
            alive = self.control_uart.is_link_alive(threshold)
        except Exception:
            alive = False
        if not alive and not self._hb_tripped:
            self._hb_tripped = True
            # Attempt to send shutdown to control board
            try:
                self.control_uart.send_shutdown()
            except Exception:
                pass
            self.log_event("Heartbeat lost: sent SHUTDOWN and marking control board disconnected.")
            try:
                # Stop background threads and close port cleanly
                try:
                    self.control_uart.stop_link_maintenance()
                except Exception:
                    pass
                self.control_uart.close()
            except Exception:
                pass
            self.control_uart = None
            # Reset UI displayed values to placeholders
            try:
                self.stim_amplitude_var.set("Stim Amplitude: — uA")
                self.pulse_width_var.set("Pulse Width: —")
            except Exception:
                pass
            self.disable_uart_controls()
            self.reconnect_but.config(state=NORMAL)

    def _event_poller(self):
        # Poll UART events and log shutdown origin
        try:
            self._evt_poll_id = self.master.after(100, self._event_poller)
            if not self.control_uart:
                return
            events = []
            try:
                events = self.control_uart.get_events(max_items=10)
            except Exception:
                events = []
            for ev in events:
                if not isinstance(ev, dict):
                    continue
                if ev.get("type") == "shutdown":
                    src = ev.get("source")
                    origin = "Unknown"
                    reason = None
                    try:
                        SRC = uart_protocol.SHUTDOWN_SRC
                        if src == getattr(SRC, 'SHUTDOWN_SRC_PI', 3):
                            origin = "Raspberry Pi"
                        elif src == getattr(SRC, 'SHUTDOWN_SRC_CONTROL', 1):
                            origin = "Control Board"
                        elif src == getattr(SRC, 'SHUTDOWN_SRC_STIM', 2):
                            origin = "Stimulator Board"
                        elif src == getattr(SRC, 'SHUTDOWN_SRC_CONTROL_PI_HEARTBEAT_LOSS', 4):
                            origin = "Control Board"
                            reason = "Raspberry Pi heartbeat lost"
                        elif src == getattr(SRC, 'SHUTDOWN_SRC_CONTROL_STIM_HEARTBEAT_LOSS', 5):
                            origin = "Control Board"
                            reason = "Stimulator Board heartbeat lost"
                        elif src == getattr(SRC, 'SHUTDOWN_SRC_STIM_CONTROL_HEARTBEAT_LOSS', 6):
                            origin = "Stimulator Board"
                            reason = "Control Board heartbeat lost"
                    except Exception:
                        pass
                    if reason:
                        self.log_event(f"Shutdown: {origin} — reason: {reason}")
                    else:
                        self.log_event(f"Shutdown message received from: {origin}")
                    # After any shutdown, enable Unlock Interlocks if still connected
                    try:
                        if self.control_uart:
                            self.interlocks_on = True
                            self.stim_on = False
                            self.update_control_button_states()
                    except Exception:
                        pass
                elif ev.get("type") == "receipt":
                    # Log echoed request receipts from control board (exclude heartbeat)
                    mt = ev.get("msg_type")
                    label = None
                    try:
                        M = uart_protocol.MODE
                        if mt == getattr(M, 'UART_MSG_UNLOCK', None):
                            label = "Control Receipt: Unlock Interlocks"
                        elif mt == getattr(M, 'UART_MSG_START_STIM', None):
                            label = "Control Receipt: Start Stim"
                        elif mt == getattr(M, 'UART_MSG_GET_PARAMS', None):
                            label = "Control Receipt: Get Params"
                        elif mt == getattr(M, 'UART_MSG_UPDATE_WIDTH', None):
                            label = "Control Receipt: Update Width"
                        elif mt == getattr(M, 'UART_MSG_UPDATE_AMP', None):
                            label = "Control Receipt: Update Amplitude"
                        elif mt == getattr(M, 'UART_MSG_UPDATE_FREQ', None):
                            label = "Control Receipt: Update Frequency"
                        elif mt == getattr(M, 'UART_MSG_UPDATE_BURST', None):
                            label = "Control Receipt: Update Burst Length"
                        elif mt == getattr(M, 'UART_MSG_SHUTDOWN', None):
                            label = "Control Receipt: Shutdown"
                    except Exception:
                        label = None
                    if label:
                        self.log_event(label)
                    else:
                        try:
                            self.log_event(f"Receipt for message type: {int(mt)}")
                        except Exception:
                            self.log_event("Receipt received")
                elif ev.get("type") == "status":
                    code = ev.get("code")
                    msg = None
                    try:
                        SC = uart_protocol.STATUS_CODE
                        prev_interlocks = self.interlocks_on
                        prev_stim = self.stim_on
                        if code == getattr(SC, 'STATUS_INTERLOCK_ON', 1):
                            msg = "Stim Status: Interlocks ON"
                            self.interlocks_on = True
                            self.stim_on = False
                            self.update_control_button_states()
                        elif code == getattr(SC, 'STATUS_INTERLOCK_OFF', 2):
                            msg = "Stim Status: Interlocks OFF"
                            self.interlocks_on = False
                            self.update_control_button_states()
                        elif code == getattr(SC, 'STATUS_STIM_ON', 3):
                            msg = "Stim Status: Stimulation ON"
                            self.stim_on = True
                            self.update_control_button_states()
                        elif code == getattr(SC, 'STATUS_STIM_OFF', 4):
                            msg = "Stim Status: Stimulation OFF"
                            self.stim_on = False
                            self.update_control_button_states()
                        elif code == getattr(SC, 'STATUS_DAC_VALUE_SET', 5):
                            msg = "Status: DAC value set"
                        elif code == getattr(SC, 'STATUS_DAC_ZEROED', 6):
                            msg = "Status: DAC zeroed"
                        elif code == getattr(SC, 'STATUS_CTRL_SHUTDOWN_RTN', 100):
                            msg = "Control Status: Shutdown routine complete"
                            # Control board reports shutdown sequence complete: system is safe/locked
                            if self.control_uart:
                                self.interlocks_on = True
                                self.stim_on = False
                                self.update_control_button_states()
                        elif code == getattr(SC, 'STATUS_CTRL_STIM_RTN', 101):
                            msg = "Control Status: Stim routine complete"
                        elif code == getattr(SC, 'STATUS_CTRL_AMP_RTN', 102):
                            msg = "Control Status: Amplitude routine complete"
                        elif code == getattr(SC, 'STATUS_CTRL_STIM_LINK_UP', 103):
                            msg = "Control Status: Stim link up"
                        elif code == getattr(SC, 'STATUS_SYSTEM_SAFE_LOCKED', 110):
                            msg = "System Status: SAFE LOCKED"
                            self.interlocks_on = True
                            self.stim_on = False
                            self.update_control_button_states()
                        elif code == getattr(SC, 'STATUS_SYSTEM_IDLE_UNLOCKED', 111):
                            msg = "System Status: IDLE UNLOCKED"
                            self.interlocks_on = False
                            self.stim_on = False
                            self.update_control_button_states()
                        elif code == getattr(SC, 'STATUS_SYSTEM_STIMULATING', 112):
                            msg = "System Status: STIMULATING"
                            self.interlocks_on = False
                            self.stim_on = True
                            self.update_control_button_states()
                        elif code == getattr(SC, 'STATUS_SYSTEM_RECORDING', 113):
                            msg = "System Status: RECORDING"
                            # Assume unlocked but not stimulating
                            self.interlocks_on = False
                            self.stim_on = False
                            self.update_control_button_states()
                    except Exception:
                        msg = None

                    # Only log when the interpreted stim/control state actually changes
                    should_log = False
                    SC = uart_protocol.STATUS_CODE
                    if code in (
                        getattr(SC, 'STATUS_INTERLOCK_ON', 1),
                        getattr(SC, 'STATUS_INTERLOCK_OFF', 2),
                    ):
                        # Interlock state change
                        if self.interlocks_on is not prev_interlocks:
                            should_log = True
                            self._last_interlocks_on = self.interlocks_on
                    elif code in (
                        getattr(SC, 'STATUS_STIM_ON', 3),
                        getattr(SC, 'STATUS_STIM_OFF', 4),
                    ):
                        # Stim on/off change
                        if self.stim_on is not prev_stim:
                            should_log = True
                            self._last_stim_on = self.stim_on
                    else:
                        # For other status codes (DAC, control/system), only
                        # log when the numeric code itself changes.
                        if code is not None and code != self._last_status_code:
                            should_log = True
                            self._last_status_code = code

                    if should_log:
                        if not msg:
                            try:
                                msg = f"Status code received: {int(code)}"
                            except Exception:
                                msg = "Status message received"
                        self.log_event(msg)

                elif ev.get("type") == "send_width":
                    width = ev.get("width")
                    if width is not None:
                        try:
                            self.pulse_width = float(width)
                            self.pending_pulse_width = float(width)
                            self.pulse_width_var.set(f"Pulse Width: {self.pulse_width:.2f}")
                            self.log_event(f"Control board width updated: {width}")
                            try:
                                self.update_plot()
                            except Exception:
                                pass
                        except Exception:
                            pass

                elif ev.get("type") == "send_amp":
                    amp = ev.get("amp")
                    if amp is not None:
                        try:
                            recv_amp = float(amp)
                            # Reflect UI policy: cap at max for display/pending
                            if recv_amp > self._max_amp_ua:
                                self.log_event(f"Control board reported amplitude {recv_amp:.2f}uA; capping display to {self._max_amp_ua:.2f}uA")
                                recv_amp = self._max_amp_ua
                            if recv_amp < 0.0:
                                recv_amp = 0.0
                            self.stim_amplitude = recv_amp
                            self.pending_stim_amplitude = recv_amp
                            self.stim_amplitude_var.set(f"Stim Amplitude: {self.stim_amplitude:.2f}uA")
                            self.log_event(f"Control board amplitude updated: {self.stim_amplitude:.2f}uA")
                            try:
                                self.update_plot()
                            except Exception:
                                pass
                        except Exception:
                            pass

                elif ev.get("type") == "send_freq":
                    freq = ev.get("freq")
                    if freq is not None:
                        try:
                            # Frequency is transported as INT32 Hz
                            self.stim_frequency = int(freq)
                            self.pending_stim_frequency = float(self.stim_frequency)
                            try:
                                self.stim_freq_entry.delete(0, END)
                                self.stim_freq_entry.insert(0, f"{self.pending_stim_frequency:.2f}")
                            except Exception:
                                pass
                            self.log_event(f"Control board frequency updated: {self.stim_frequency} Hz")
                            try:
                                self.update_plot()
                            except Exception:
                                pass
                        except Exception:
                            pass

                elif ev.get("type") == "send_burst":
                    burst = ev.get("burst")
                    if burst is not None:
                        try:
                            # Burst length is transported as INT32 microseconds
                            self.burst_length = int(burst)
                            self.pending_burst_length = float(self.burst_length)
                            try:
                                self.burst_len_entry.delete(0, END)
                                self.burst_len_entry.insert(0, f"{self.pending_burst_length:.2f}")
                            except Exception:
                                pass
                            self.log_event(f"Control board burst length updated: {self.burst_length} us")
                        except Exception:
                            pass
        except Exception:
            pass

    # def initialise_user_board(self):
    #     """Initialize the connection to the user board."""
    #     port = self.find_user_usb_port()
    #     if not port:
    #         self.log_event("Error: User board not found (expecting single USB serial device). Please check the connection.")
    #         self.reconnect_user_but.config(state=NORMAL)  # Enable reconnect button for user board
    #         return

    #     try:
    #         self.user_board = USER_COMMS(port=port, baudrate=9600)
    #         self.reconnect_user_but.config(state=DISABLED)  # Disable reconnect button for user board
    #         self.pc_user_switch.config(state=NORMAL)  # Enable PC/User switch
    #         self.user_board_serial = port
    #         self.log_event(f"User board connected on port {port}.")
    #     except Exception as e:
    #         self.log_event(f"Failed to connect to user board: {e}")
    #         self.reconnect_user_but.config(state=NORMAL)  # Enable reconnect button for user board
    #         self.pc_user_switch.config(state=DISABLED)  # Disable PC/User switch

    def log_event(self, event):
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.event_log.insert(END, f"{timestamp} - {event}\n")
        self.event_log.see(END)

    def toggle_debug(self):
        """Show or hide the debug log panel based on the Show Debug toggle."""
        try:
            if self.show_debug_var.get():
                # Show only the log content; keep checkbox always visible
                try:
                    self.event_log.grid(row=0, column=0, padx=(0, 4), sticky="ew")
                except Exception:
                    pass
                try:
                    self.log_scroll.grid(row=0, column=1, sticky="ns")
                except Exception:
                    pass
            else:
                # Hide the log content but leave the frame and checkbox visible
                try:
                    self.event_log.grid_remove()
                except Exception:
                    pass
                try:
                    self.log_scroll.grid_remove()
                except Exception:
                    pass
        except Exception:
            pass

    def update_plot(self):
        """Render two square-wave pulses on the right-side canvas.
        X axis: time (s), Y axis: stim amplitude (uA).
        Two pulses each of width `pending_pulse_width`, amplitude `pending_stim_amplitude`,
        separated by a gap of 1/`pending_stim_frequency`.
        """
        canvas = getattr(self, "plot_canvas", None)
        if not canvas:
            return

        # Clear previous drawing
        try:
            canvas.delete("all")
        except Exception:
            return

        # Fetch parameters (use pending so user sees edits immediately)
        try:
            amp = float(getattr(self, "pending_stim_amplitude", 0.0) or 0.0)
        except Exception:
            amp = 0.0
        try:
            width = float(getattr(self, "pending_pulse_width", 0.0) or 0.0)
        except Exception:
            width = 0.0
        try:
            freq = float(getattr(self, "pending_stim_frequency", 0.0) or 0.0)
        except Exception:
            freq = 0.0

        # Guard and defaults
        if width < 0:
            width = 0.0
        gap = 1.0 / freq if freq and freq > 0 else max(1.0, width)

        # Time domain: first pulse [0, width], gap, second pulse [width+gap, width+gap+width]
        t0 = 0.0
        t1 = width
        t2 = width + gap
        t3 = width + gap + width
        t_max = max(t3, 1.0)

        # Canvas size
        try:
            W = int(canvas.winfo_width()) or 420
            H = int(canvas.winfo_height()) or 300
        except Exception:
            W, H = 420, 300

        # Margins
        L, R, T, B = 40, 10, 10, 30
        plot_w = max(1, W - L - R)
        plot_h = max(1, H - T - B)

        # Axes and labels
        x0, y0 = L, H - B
        x1, y1 = W - R, T
        canvas.create_rectangle(x0, y1, x1, y0, outline="#999999")
        canvas.create_text((x0 + x1) // 2, H - 8, text="Time (s)", fill="#333333", font=("TkDefaultFont", 9))
        canvas.create_text(14, (y1 + y0) // 2, text="uA", fill="#333333", angle=90, font=("TkDefaultFont", 9))

        # Scaling functions
        def x_to_px(t):
            return x0 + (0 if t_max == 0 else (t / t_max) * plot_w)

        # Fixed Y-axis at 10uA
        y_max = 10.0
        def y_to_px(v):
            return y0 - (0 if y_max == 0 else (v / y_max) * plot_h)

        # Draw baseline
        canvas.create_line(x_to_px(0), y_to_px(0), x_to_px(t_max), y_to_px(0), fill="#bbbbbb")

        # Draw pulses as filled rectangles
        fill_col = "#4a90e2"
        outline_col = "#2c66a8"
        if width > 0 and amp > 0:
            # First pulse
            canvas.create_rectangle(x_to_px(t0), y_to_px(0), x_to_px(t1), y_to_px(amp), outline=outline_col, fill=fill_col)
            # Second pulse
            canvas.create_rectangle(x_to_px(t2), y_to_px(0), x_to_px(t3), y_to_px(amp), outline=outline_col, fill=fill_col)

        # Simple ticks (3 ticks for time, 3 for amplitude)
        for i in range(4):
            tx = i * (t_max / 3.0)
            px = x_to_px(tx)
            canvas.create_line(px, y0, px, y0 + 4, fill="#666666")
            canvas.create_text(px, y0 + 12, text=f"{tx:.2f}", fill="#444444", font=("TkDefaultFont", 8))

        for i in range(4):
            tv = i * (y_max / 3.0)
            py = y_to_px(tv)
            canvas.create_line(x0 - 4, py, x0, py, fill="#666666")
            canvas.create_text(x0 - 8, py, text=f"{tv:.0f}", fill="#444444", font=("TkDefaultFont", 8), anchor="e")

    def stim_amp_up(self):
        self.pending_stim_amplitude += 1.00
        if self.pending_stim_amplitude > self._max_amp_ua:
            self.pending_stim_amplitude = self._max_amp_ua
        self.stim_amplitude_var.set(f"Stim Amplitude: {self.pending_stim_amplitude:.2f}uA")
        try:
            self.update_plot()
        except Exception:
            pass

    def stim_amp_down(self):
        self.pending_stim_amplitude -= 1.00
        if self.pending_stim_amplitude < 0.0:
            self.pending_stim_amplitude = 0.0
        self.stim_amplitude_var.set(f"Stim Amplitude: {self.pending_stim_amplitude:.2f}uA")
        try:
            self.update_plot()
        except Exception:
            pass

    def pulse_width_up(self):
        self.pending_pulse_width += 1.00
        self.pulse_width_var.set(f"Pulse Width: {self.pending_pulse_width:.2f}")
        try:
            self.update_plot()
        except Exception:
            pass

    def pulse_width_down(self):
        self.pending_pulse_width -= 1.00
        if self.pending_pulse_width < 0:
            self.pending_pulse_width = 0
        self.pulse_width_var.set(f"Pulse Width: {self.pending_pulse_width:.2f}")
        try:
            self.update_plot()
        except Exception:
            pass

    def stim_freq_up(self):
        """Increase pending stimulation frequency and update entry."""
        self.pending_stim_frequency += 1.00
        try:
            self.stim_freq_entry.delete(0, END)
            self.stim_freq_entry.insert(0, f"{self.pending_stim_frequency:.2f}")
        except Exception:
            pass
        try:
            self.update_plot()
        except Exception:
            pass

    def stim_freq_down(self):
        """Decrease pending stimulation frequency and update entry."""
        self.pending_stim_frequency -= 1.00
        if self.pending_stim_frequency < 0:
            self.pending_stim_frequency = 0
        try:
            self.stim_freq_entry.delete(0, END)
            self.stim_freq_entry.insert(0, f"{self.pending_stim_frequency:.2f}")
        except Exception:
            pass
        try:
            self.update_plot()
        except Exception:
            pass

    def burst_len_up(self):
        """Increase pending burst length and update entry."""
        self.pending_burst_length += 1.00
        try:
            self.burst_len_entry.delete(0, END)
            self.burst_len_entry.insert(0, f"{self.pending_burst_length:.2f}")
        except Exception:
            pass

    def burst_len_down(self):
        """Decrease pending burst length and update entry."""
        self.pending_burst_length -= 1.00
        if self.pending_burst_length < 0:
            self.pending_burst_length = 0
        try:
            self.burst_len_entry.delete(0, END)
            self.burst_len_entry.insert(0, f"{self.pending_burst_length:.2f}")
        except Exception:
            pass

    def apply_settings(self):
        """Apply the pending stimulation settings (amplitude, width, freq, burst).

        Frequency and burst length are normalised to INT32 values before
        being sent to the control board so that all parameters on the wire
        are integer or float32 scalars as defined by the protocol.
        """
        # Clamp amplitude before applying
        if self.pending_stim_amplitude > self._max_amp_ua:
            self.log_event(f"Amplitude request exceeds limit; clipping to {self._max_amp_ua:.2f}uA")
            self.pending_stim_amplitude = self._max_amp_ua
        if self.pending_stim_amplitude < 0.0:
            self.pending_stim_amplitude = 0.0
        self.stim_amplitude = self.pending_stim_amplitude
        self.pulse_width = self.pending_pulse_width
        # Normalise frequency and burst length to integer values for transport
        try:
            self.stim_frequency = int(self.pending_stim_frequency)
        except Exception:
            self.stim_frequency = 0
        try:
            self.burst_length = int(self.pending_burst_length)
        except Exception:
            self.burst_length = 0

        self.log_event(
            f"Applied settings: Stim Amplitude = {self.stim_amplitude:.2f}uA, "
            f"Pulse Width = {self.pulse_width:.2f}, Freq = {self.stim_frequency} Hz, "
            f"Burst = {self.burst_length} us"
        )
        try:
            self.update_plot()
        except Exception:
            pass
        if not self.control_uart:
            self.log_event("Cannot apply settings: control board not connected.")
            return
        try:
            # Send width (int), amplitude (float), frequency (int), and burst (int)
            # to the control board. Frequency and burst are sent as INT32 payloads.
            self.control_uart.set_pulse_width(self.pulse_width)
            # Final guard before sending
            send_amp = self.stim_amplitude
            if send_amp > self._max_amp_ua:
                self.log_event(f"Safety check: capping amplitude to {self._max_amp_ua:.2f}uA before send")
                send_amp = self._max_amp_ua
            if send_amp < 0.0:
                send_amp = 0.0
            self.control_uart.set_stim_amplitude(send_amp)
            self.control_uart.set_stim_frequency(self.stim_frequency)
            self.control_uart.set_burst_length(self.burst_length)
            self.log_event("Settings sent to control board.")
        except Exception as e:
            self.log_event(f"Failed to send settings: {e}")

    def STOP(self):
        # Send SHUTDOWN message without closing connections or window
        if not self.control_uart:
            self.log_event("Cannot STOP: control board not connected.")
            return
        try:
            self.control_uart.send_shutdown()
            self.log_event("STOP: sent SHUTDOWN to control board.")
        except Exception as e:
            self.log_event(f"STOP: failed to send SHUTDOWN: {e}")

    def toggle_trigger(self):
        """Handle trigger source selection between Internal and External.

        Sends a trigger-mode command to the control board, which forwards it
        to the stim board to toggle its internal_triggers flag. When
        External is selected, the stim board disables any internally
        generated pulses from the stored stim parameters.
        """
        if not self.control_uart:
            self.log_event("Cannot change trigger mode: control board not connected.")
            return

        try:
            mode_val = self.trigger_mode_var.get()
        except Exception:
            mode_val = 0

        internal = (mode_val == 0)
        try:
            self.control_uart.set_trigger_mode(internal)
        except Exception as e:
            self.log_event(f"Failed to send trigger mode: {e}")

        if internal:
            self.log_event("INTERNAL TRIGGERS")
        else:
            self.log_event("EXTERNAL TRIGGERS")

    def toggle_recording(self):
        """Handle mode selection change between Recording and Stimulation.

        Sends an explicit STIM/RECORD message to the control board and
        updates the UI layout to reflect the chosen mode.
        """
        if not self.control_uart:
            self.log_event("Cannot change mode: control board not connected.")
            return

        try:
            mode_val = self.mode_var.get()
        except Exception:
            mode_val = 0

        try:
            if mode_val == 0:
                # Recording mode
                self.control_uart.set_mode_record()
                self.log_event("Recording Mode Enabled")
                self.recording_ui()
            else:
                # Stimulation mode
                self.control_uart.set_mode_stim()
                self.log_event("Stimulation Mode Enabled")
                self.stimulation_ui()
        except Exception as e:
            self.log_event(f"Failed to send mode change: {e}")

    def toggle_pc_user(self):
        if not self.user_board:
            self.log_event("Cannot toggle PC/User mode: User board not connected")
            return
        # Aesthetic-only change: switch UI between User/PC layouts based on
        # radio selection, without altering underlying messaging behaviour.
        try:
            mode_val = self.pc_user_mode_var.get()
        except Exception:
            mode_val = 0

        if mode_val == 1:
            self.log_event("PC Mode Enabled")
            self.pc_mode_ui()
        else:
            self.log_event("User Mode Enabled")
            self.user_mode_ui()

    def recording_ui(self):
        self.stim_up_but.config(state=DISABLED)
        self.stim_down_but.config(state=DISABLED)
        self.pulse_up_but.config(state=DISABLED)
        self.pulse_down_but.config(state=DISABLED)
        self.stim_freq_up_but.config(state=DISABLED)
        self.stim_freq_down_but.config(state=DISABLED)
        self.burst_len_up_but.config(state=DISABLED)
        self.burst_len_down_but.config(state=DISABLED)
        self.stim_amplitude_entry.config(state=DISABLED)
        self.pulse_width_entry.config(state=DISABLED)
        self.stim_freq_entry.config(state=DISABLED)
        self.burst_len_entry.config(state=DISABLED)
        try:
            self.trigger_internal_radio.config(state=DISABLED)
            self.trigger_external_radio.config(state=DISABLED)
        except Exception:
            pass
        try:
            self.pc_mode_user_radio.config(state=DISABLED)
            self.pc_mode_pc_radio.config(state=DISABLED)
        except Exception:
            pass
        self.done_but.config(state=DISABLED)
        # In recording mode, stimulation cannot be started
        try:
            self.start_stim_but.config(state=DISABLED, text="Start Stim")
        except Exception:
            pass

    def stimulation_ui(self):
        self.stim_up_but.config(state=NORMAL)
        self.stim_down_but.config(state=NORMAL)
        self.pulse_up_but.config(state=NORMAL)
        self.pulse_down_but.config(state=NORMAL)
        self.stim_freq_up_but.config(state=NORMAL)
        self.stim_freq_down_but.config(state=NORMAL)
        self.burst_len_up_but.config(state=NORMAL)
        self.burst_len_down_but.config(state=NORMAL)
        self.stim_amplitude_entry.config(state=NORMAL)
        self.pulse_width_entry.config(state=NORMAL)
        self.stim_freq_entry.config(state=NORMAL)
        self.burst_len_entry.config(state=NORMAL)
        try:
            self.trigger_internal_radio.config(state=NORMAL)
            self.trigger_external_radio.config(state=NORMAL)
        except Exception:
            pass
        try:
            self.pc_mode_user_radio.config(state=NORMAL)
            self.pc_mode_pc_radio.config(state=NORMAL)
        except Exception:
            pass
        self.poll_status_but.config(state=NORMAL)
        self.start_but.config(state=NORMAL)
        # Start Stim availability is still subject to interlocks/mode in
        # update_control_button_states; default to disabled until STATUS.
        try:
            self.start_stim_but.config(state=DISABLED, text="Start Stim")
        except Exception:
            pass

    def pc_mode_ui(self):
        self.stim_up_but.config(state=DISABLED)
        self.stim_down_but.config(state=DISABLED)
        self.pulse_up_but.config(state=DISABLED)
        self.pulse_down_but.config(state=DISABLED)
        self.stim_freq_up_but.config(state=DISABLED)
        self.stim_freq_down_but.config(state=DISABLED)
        self.burst_len_up_but.config(state=DISABLED)
        self.burst_len_down_but.config(state=DISABLED)
        self.stim_amplitude_entry.config(state=DISABLED)
        self.pulse_width_entry.config(state=DISABLED)
        self.stim_freq_entry.config(state=DISABLED)
        self.burst_len_entry.config(state=DISABLED)
        try:
            self.trigger_internal_radio.config(state=DISABLED)
            self.trigger_external_radio.config(state=DISABLED)
        except Exception:
            pass
        self._set_mode_controls_state(DISABLED)
        self.poll_status_but.config(state=DISABLED)
        self.start_but.config(state=DISABLED)
        self.start_stim_but.config(state=DISABLED)

    def user_mode_ui(self):
        self.stim_up_but.config(state=NORMAL)
        self.stim_down_but.config(state=NORMAL)
        self.pulse_up_but.config(state=NORMAL)
        self.pulse_down_but.config(state=NORMAL)
        self.stim_freq_up_but.config(state=NORMAL)
        self.stim_freq_down_but.config(state=NORMAL)
        self.burst_len_up_but.config(state=NORMAL)
        self.burst_len_down_but.config(state=NORMAL)
        self.stim_amplitude_entry.config(state=NORMAL)
        self.pulse_width_entry.config(state=NORMAL)
        self.stim_freq_entry.config(state=NORMAL)
        self.burst_len_entry.config(state=NORMAL)
        try:
            self.trigger_internal_radio.config(state=NORMAL)
            self.trigger_external_radio.config(state=NORMAL)
        except Exception:
            pass
        self._set_mode_controls_state(NORMAL)
        self.start_but.config(state=NORMAL)
        self.start_stim_but.config(state=NORMAL)

    def update_stim_amplitude(self, event):
        """Update the pending stimulation amplitude from the entry box."""
        try:
            value = float(self.stim_amplitude_entry.get())
            # Clamp to [0, max]
            if value < 0.0:
                value = 0.0
            if value > self._max_amp_ua:
                value = self._max_amp_ua
            self.pending_stim_amplitude = value
            self.stim_amplitude_var.set(f"Stim Amplitude: {self.pending_stim_amplitude:.2f}uA")
        except ValueError:
            self.log_event("Invalid input for Stimulation Amplitude")
        try:
            self.update_plot()
        except Exception:
            pass

    def update_pulse_width(self, event):
        """Update the pending pulse width from the entry box."""
        try:
            value = float(self.pulse_width_entry.get())
            self.pending_pulse_width = value
            self.pulse_width_var.set(f"Pulse Width: {self.pending_pulse_width:.2f}")
        except ValueError:
            self.log_event("Invalid input for Pulse Width")
        try:
            self.update_plot()
        except Exception:
            pass

    def update_stim_freq(self, event):
        """Update the pending stimulation frequency from the entry box."""
        try:
            value = float(self.stim_freq_entry.get())
            self.pending_stim_frequency = value
        except ValueError:
            self.log_event("Invalid input for Stim Frequency")
        try:
            self.update_plot()
        except Exception:
            pass

    def update_burst_len(self, event):
        """Update the pending burst length from the entry box."""
        try:
            value = float(self.burst_len_entry.get())
            self.pending_burst_length = value
        except ValueError:
            self.log_event("Invalid input for Burst Length")

    def disable_ui(self):
        self.stim_up_but.config(state=DISABLED)
        self.stim_down_but.config(state=DISABLED)
        self.pulse_up_but.config(state=DISABLED)
        self.pulse_down_but.config(state=DISABLED)
        self.stim_freq_up_but.config(state=DISABLED)
        self.stim_freq_down_but.config(state=DISABLED)
        self.burst_len_up_but.config(state=DISABLED)
        self.burst_len_down_but.config(state=DISABLED)
        self.stim_amplitude_entry.config(state=DISABLED)
        self.pulse_width_entry.config(state=DISABLED)
        self.stim_freq_entry.config(state=DISABLED)
        self.burst_len_entry.config(state=DISABLED)
        try:
            self.trigger_internal_radio.config(state=DISABLED)
            self.trigger_external_radio.config(state=DISABLED)
        except Exception:
            pass
        self._set_mode_controls_state(DISABLED)
        try:
            self.pc_mode_user_radio.config(state=DISABLED)
            self.pc_mode_pc_radio.config(state=DISABLED)
        except Exception:
            pass
        self.STOP_but.config(state=DISABLED)
        self.done_but.config(state=DISABLED)
        self.start_stim_but.config(state=DISABLED)

    def disable_uart_controls(self):
        """Disable only the controls that require the control board (UART)."""
        self.stim_up_but.config(state=DISABLED)
        self.stim_down_but.config(state=DISABLED)
        self.pulse_up_but.config(state=DISABLED)
        self.pulse_down_but.config(state=DISABLED)
        self.stim_freq_up_but.config(state=DISABLED)
        self.stim_freq_down_but.config(state=DISABLED)
        self.burst_len_up_but.config(state=DISABLED)
        self.burst_len_down_but.config(state=DISABLED)
        self.stim_amplitude_entry.config(state=DISABLED)
        self.pulse_width_entry.config(state=DISABLED)
        self.stim_freq_entry.config(state=DISABLED)
        self.burst_len_entry.config(state=DISABLED)
        try:
            self.trigger_internal_radio.config(state=DISABLED)
            self.trigger_external_radio.config(state=DISABLED)
        except Exception:
            pass
        self._set_mode_controls_state(DISABLED)
        # PC/User control depends on user board; keep it disabled here
        try:
            self.pc_mode_user_radio.config(state=DISABLED)
            self.pc_mode_pc_radio.config(state=DISABLED)
        except Exception:
            pass
        self.STOP_but.config(state=DISABLED)
        self.done_but.config(state=DISABLED)
        self.poll_status_but.config(state=DISABLED)
        self.start_but.config(state=DISABLED)
        self.start_stim_but.config(state=DISABLED)

    def enable_uart_controls(self):
        """Enable only the controls that require the control board (UART)."""
        self.stim_up_but.config(state=NORMAL)
        self.stim_down_but.config(state=NORMAL)
        self.pulse_up_but.config(state=NORMAL)
        self.pulse_down_but.config(state=NORMAL)
        self.stim_freq_up_but.config(state=NORMAL)
        self.stim_freq_down_but.config(state=NORMAL)
        self.burst_len_up_but.config(state=NORMAL)
        self.burst_len_down_but.config(state=NORMAL)
        self.stim_amplitude_entry.config(state=NORMAL)
        self.pulse_width_entry.config(state=NORMAL)
        self.stim_freq_entry.config(state=NORMAL)
        self.burst_len_entry.config(state=NORMAL)
        try:
            self.trigger_internal_radio.config(state=NORMAL)
            self.trigger_external_radio.config(state=NORMAL)
        except Exception:
            pass
        self._set_mode_controls_state(NORMAL)
        # Only enable PC/User control if user board is present
        if getattr(self, 'user_board', None):
            try:
                self.pc_mode_user_radio.config(state=NORMAL)
                self.pc_mode_pc_radio.config(state=NORMAL)
            except Exception:
                pass
        else:
            try:
                self.pc_mode_user_radio.config(state=DISABLED)
                self.pc_mode_pc_radio.config(state=DISABLED)
            except Exception:
                pass
        self.STOP_but.config(state=NORMAL)
        self.done_but.config(state=NORMAL)
        self.poll_status_but.config(state=NORMAL)
        self.update_control_button_states()

    def update_control_button_states(self):
        """Enable/disable control action buttons based on interlock/stim state."""
        # Interlocks determine availability of Unlock vs Start Stim
        if self.interlocks_on is True:
            # Interlocks engaged: cannot start stim; can unlock
            try:
                self.start_stim_but.config(state=DISABLED, text="Start Stim")
            except Exception:
                pass
            try:
                self.start_but.config(state=NORMAL)
            except Exception:
                pass
        elif self.interlocks_on is False:
            # Interlocks disengaged: can start stim; cannot unlock again
            try:
                self.start_but.config(state=DISABLED)
            except Exception:
                pass
            try:
                # Enable Start/Stop based on stim state
                self.start_stim_but.config(state=NORMAL)
            except Exception:
                pass
        else:
            # Unknown: conservative defaults (allow unlock, block start stim)
            try:
                self.start_but.config(state=NORMAL)
            except Exception:
                pass
            try:
                self.start_stim_but.config(state=DISABLED, text="Start Stim")
            except Exception:
                pass

        # Label reflects stim state
        try:
            if self.stim_on is True:
                self.start_stim_but.config(text="Stop Stim")
            else:
                self.start_stim_but.config(text="Start Stim")
        except Exception:
            pass

        # Enforce additional gating based on selected mode and active stimulation.
        try:
            # If mode_var exists, 0 = Recording, 1 = Stimulation
            if getattr(self, "mode_var", None) is not None:
                mode_val = self.mode_var.get()

                # In recording mode, never allow Start Stim to be pressed
                if mode_val == 0:
                    try:
                        self.start_stim_but.config(state=DISABLED, text="Start Stim")
                    except Exception:
                        pass

                # While stimulation is active, prevent changing mode from STIM to RECORD
                # Only adjust mode controls in User mode; PC/User helpers manage their own state.
                in_pc_mode = False
                try:
                    if getattr(self, "pc_user_switch_var", None) is not None:
                        in_pc_mode = bool(self.pc_user_switch_var.get())
                except Exception:
                    in_pc_mode = False

                if self.stim_on is True:
                    if not in_pc_mode:
                        self._set_mode_controls_state(DISABLED)
                else:
                    # When not actively stimulating, allow mode changes in User mode
                    if not in_pc_mode:
                        self._set_mode_controls_state(NORMAL)
        except Exception:
            pass

    def start_system(self):
        if not self.control_uart:
            self.log_event("Cannot Unlock Interlocks: control board not connected.")
            return
        try:
            self.control_uart.send_unlock()
            self.log_event("Unlocking Interlocks.")
            # Await confirmation via STATUS before re-enabling
            try:
                self.start_but.config(state=DISABLED)
                self.start_stim_but.config(state=DISABLED)
            except Exception:
                pass
        except Exception as e:
            self.log_event(f"Unlocking Interlocks: failed to send command: {e}")

    def start_stim(self):
        if not self.control_uart:
            self.log_event("Cannot Start/Stop Stim: control board not connected.")
            return
        try:
            # Do not allow stimulation to be started from the UI when in Recording mode
            try:
                if getattr(self, "mode_var", None) is not None and self.mode_var.get() == 0 and self.stim_on is not True:
                    self.log_event("Start Stim ignored: Recording mode is active.")
                    return
            except Exception:
                pass

            if self.stim_on is True:
                # Treat button as Stop Stim when active
                try:
                    self.control_uart.send_shutdown()
                    self.log_event("Stop Stim command sent.")
                except Exception as e:
                    self.log_event(f"Stop Stim: failed to send command: {e}")
            else:
                self.control_uart.send_start_stim()
                self.log_event("Start Stim command sent.")
            # Disable until confirmation via STATUS
            try:
                self.start_stim_but.config(state=DISABLED)
            except Exception:
                pass
        except Exception as e:
            self.log_event(f"Stim command failed: {e}")

    def enable_ui(self):
        self.stim_up_but.config(state=NORMAL)
        self.stim_down_but.config(state=NORMAL)
        self.pulse_up_but.config(state=NORMAL)
        self.pulse_down_but.config(state=NORMAL)
        self.stim_freq_up_but.config(state=NORMAL)
        self.stim_freq_down_but.config(state=NORMAL)
        self.burst_len_up_but.config(state=NORMAL)
        self.burst_len_down_but.config(state=NORMAL)
        self.stim_amplitude_entry.config(state=NORMAL)
        self.pulse_width_entry.config(state=NORMAL)
        self.stim_freq_entry.config(state=NORMAL)
        self.burst_len_entry.config(state=NORMAL)
        try:
            self.trigger_internal_radio.config(state=NORMAL)
            self.trigger_external_radio.config(state=NORMAL)
        except Exception:
            pass
        self._set_mode_controls_state(NORMAL)
        try:
            self.pc_mode_user_radio.config(state=NORMAL)
            self.pc_mode_pc_radio.config(state=NORMAL)
        except Exception:
            pass
        self.STOP_but.config(state=NORMAL)
        self.done_but.config(state=NORMAL)
        self.poll_status_but.config(state=NORMAL)
        self.start_stim_but.config(state=NORMAL)

    def close(self):
        # Graceful close when window is closed
        try:
            self._cancel_scheduled_callbacks()
        except Exception:
            pass

        # Close user serial if present
        try:
            if self.user_board:
                self.user_board.close()
        except Exception:
            pass
        finally:
            self.user_board = None

        # Close peer-UI USB link if present
        try:
            if self.peer_ui_ser and getattr(self.peer_ui_ser, "is_open", False):
                self.peer_ui_ser.close()
        except Exception:
            pass
        finally:
            self.peer_ui_ser = None
            self._peer_ui_connected = False

        # Send SHUTDOWN, stop UART threads, and close serial if present
        try:
            if self.control_uart:
                try:
                    self.control_uart.send_shutdown()
                except Exception:
                    pass
                try:
                    self.control_uart.stop_link_maintenance()
                except Exception:
                    pass
                try:
                    self.control_uart.close()
                except Exception:
                    pass
        except Exception:
            pass
        finally:
            self.control_uart = None

        # Finally destroy the window
        try:
            self.master.destroy()
        except Exception:
            pass

    def _cancel_scheduled_callbacks(self):
        # Helper to cancel Tk after() callbacks for connect/heartbeat loops
        try:
            if getattr(self, '_hb_monitor_id', None) is not None:
                try:
                    self.master.after_cancel(self._hb_monitor_id)
                except Exception:
                    pass
                self._hb_monitor_id = None
        except Exception:
            pass
        try:
            if getattr(self, '_evt_poll_id', None) is not None:
                try:
                    self.master.after_cancel(self._evt_poll_id)
                except Exception:
                    pass
                self._evt_poll_id = None
        except Exception:
            pass
        try:
            if getattr(self, '_connect_after_id', None) is not None:
                try:
                    self.master.after_cancel(self._connect_after_id)
                except Exception:
                    pass
                self._connect_after_id = None
        except Exception:
            pass
        try:
            if getattr(self, '_mode_resend_id', None) is not None:
                try:
                    self.master.after_cancel(self._mode_resend_id)
                except Exception:
                    pass
                self._mode_resend_id = None
        except Exception:
            pass

    def _set_mode_controls_state(self, state):
        """Enable/disable both Recording and Stimulation mode controls."""
        try:
            self.mode_record_radio.config(state=state)
        except Exception:
            pass
        try:
            self.mode_stim_radio.config(state=state)
        except Exception:
            pass
    
    def poll_status(self):
        """Request current parameters via GET_PARAMS and log action."""
        if self.control_uart:
            try:
                self.control_uart.send_get_params()
                self.log_event("Requested parameter sync. Waiting for response...")
            except Exception as e:
                self.log_event(f"Failed to request params: {e}")
        else:
            self.log_event("Cannot request params: control board not connected.")

    def check_user_board_connection(self):
        """Check if the user board is still connected."""
        port = self.find_user_usb_port()
        # If a different USB device list is present (zero or multiple), treat as disconnected
        if not port and self.user_board:
            self.log_event("User board disconnected.")
            try:
                self.user_board.close()
            except Exception:
                pass
            self.user_board = None
            self.pc_user_switch.config(state=DISABLED)
            self.reconnect_user_but.config(state=NORMAL)

    def _compute_peer_crc(self, msg_type: int, payload_type: int, payload_bytes: bytes) -> int:
        crc = msg_type ^ payload_type ^ len(payload_bytes)
        for b in payload_bytes:
            crc ^= b
        return crc & 0xFF

    def _read_peer_packet(self, timeout_s: float = 0.5):
        """Read a single framed UART packet from the peer-UI link.

        This mirrors the framing used elsewhere in the system
        (UART_START_BYTE / UART_END_BYTE with CRC) but is scoped to
        the simple ACK-based handshake for the USB link between the
        two UIs.
        """
        if not self.peer_ui_ser or not getattr(self.peer_ui_ser, "is_open", False):
            return None

        start_byte = getattr(uart_protocol, "UART_START_BYTE", None)
        end_byte = getattr(uart_protocol, "UART_END_BYTE", None)
        if start_byte is None or end_byte is None:
            return None

        deadline = time.monotonic() + max(0.0, timeout_s)
        while time.monotonic() < deadline:
            start = self.peer_ui_ser.read(1)
            if not start or start[0] != start_byte:
                continue

            header = self.peer_ui_ser.read(3)
            if len(header) != 3:
                continue
            msg_type, payload_type, length = header

            payload = self.peer_ui_ser.read(length)
            if len(payload) != length:
                continue

            tail = self.peer_ui_ser.read(2)
            if len(tail) != 2:
                continue
            crc_rx, end = tail
            if end != end_byte:
                continue

            if self._compute_peer_crc(msg_type, payload_type, payload) != crc_rx:
                continue

            return msg_type, payload_type, payload
        return None

    def connect_peer_ui(self, overall_timeout_s: float = 1.0, resend_interval_s: float = 0.25) -> bool:
        """Connect to the peer UI over USB serial and perform an ACK handshake.

        Uses uart_protocol framing with MODE.UART_MSG_ACK and PAYLOAD_NONE
        so both UIs can confirm that they see each other's connection.
        """
        if not self.peer_ui_serial:
            return False

        # If already connected and port is open, treat as success
        if self.peer_ui_ser is not None and getattr(self.peer_ui_ser, "is_open", False):
            self._peer_ui_connected = True
            return True

        try:
            self.log_event(f"Attempting peer-UI USB link on {self.peer_ui_serial}...")
        except Exception:
            pass

        try:
            self.peer_ui_ser = serial.Serial(
                port=self.peer_ui_serial,
                baudrate=self.peer_ui_baudrate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=0.2,
            )
        except Exception as e:
            try:
                self.log_event(f"Peer-UI link failed: unable to open {self.peer_ui_serial}: {e}")
            except Exception:
                pass
            self.peer_ui_ser = None
            self._peer_ui_connected = False
            return False

        # Build a bare ACK packet for handshake
        try:
            MODE = uart_protocol.MODE
            PAYLOAD_TYPE = uart_protocol.PAYLOAD_TYPE
            start_byte = uart_protocol.UART_START_BYTE
            end_byte = uart_protocol.UART_END_BYTE
        except Exception:
            # If protocol constants are unavailable, close and abort
            try:
                self.peer_ui_ser.close()
            except Exception:
                pass
            self.peer_ui_ser = None
            self._peer_ui_connected = False
            return False

        msg_type = MODE.UART_MSG_ACK
        payload_type = PAYLOAD_TYPE.PAYLOAD_NONE
        payload = b""
        length = len(payload)
        crc = self._compute_peer_crc(msg_type, payload_type, payload)

        ack_pkt = bytearray()
        ack_pkt.append(start_byte)
        ack_pkt.append(msg_type)
        ack_pkt.append(payload_type)
        ack_pkt.append(length)
        ack_pkt.extend(payload)
        ack_pkt.append(crc)
        ack_pkt.append(end_byte)
        ack_pkt = bytes(ack_pkt)

        start_time = time.monotonic()
        last_send = start_time

        try:
            while (time.monotonic() - start_time) < overall_timeout_s:
                # Try to read a packet from the peer
                pkt = None
                try:
                    pkt = self._read_peer_packet(timeout_s=0.2)
                except Exception:
                    pkt = None

                if pkt and pkt[0] == MODE.UART_MSG_ACK:
                    # Echo ACK back so the peer can also confirm the link
                    try:
                        self.peer_ui_ser.write(ack_pkt)
                    except Exception:
                        pass
                    self._peer_ui_connected = True
                    try:
                        self.log_event(f"Peer-UI handshake complete on {self.peer_ui_serial}.")
                    except Exception:
                        pass
                    return True

                now = time.monotonic()
                if (now - last_send) >= resend_interval_s:
                    try:
                        self.peer_ui_ser.write(ack_pkt)
                    except Exception:
                        pass
                    last_send = now

                time.sleep(0.05)
        except Exception:
            pass

        # Handshake failed; close link
        try:
            self.log_event(f"Peer-UI handshake failed on {self.peer_ui_serial}.")
        except Exception:
            pass
        try:
            if self.peer_ui_ser and getattr(self.peer_ui_ser, "is_open", False):
                self.peer_ui_ser.close()
        except Exception:
            pass
        self.peer_ui_ser = None
        self._peer_ui_connected = False
        return False

    def uart_handshake(self):
        """Send a short handshake (<ACK>) over UART and return the raw response bytes.

        Returns the raw response bytes on success or None on failure/timeout/exception.
        This centralizes write/read logic so callers can handle logging and disconnects.
        """
        if not self.uart:
            return None
        try:
            # Use UART_COMMS interface
            self.uart.send_stim_ack()
            type, _, __ = self.uart.read()
            if type is UART_COMMS.MSG_TYPE.ACK:
                return true
            else:
                return false
        except Exception:
            return None

    def handle_uart_disconnect(self, reason=None):
        """Centralized handling when UART comms are lost.

        Closes the UART object (if present), disables UART-dependent controls,
        resets failure counters, and enables the reconnect button.
        """
        msg = "UART comms disconnected."
        if reason:
            msg = f"UART comms disconnected: {reason}"
        self.log_event(msg)
        # Legacy UART object (if present) is no longer used in the
        # current control-board architecture, but close it defensively.
        try:
            if getattr(self, 'uart', None):
                self.uart.close()
        except Exception:
            pass
        self.uart = None
        self.disable_uart_controls()
        self.reconnect_but.config(state=NORMAL)

    def shutdown_system(self):
        """Close comms safely and shut down the system after confirmation."""
        confirm = messagebox.askyesno("Shutdown", "This will close serial connections and shut down the system. Continue?")
        if not confirm:
            return

        self.log_event("Shutdown requested: closing serial connections...")
        try:
            if self.user_board:
                try:
                    self.user_board.close()
                except Exception:
                    pass
                self.user_board = None
        except Exception:
            pass

        try:
            if self.control_uart:
                try:
                    # Best-effort shutdown message, then stop threads and close
                    try:
                        self.control_uart.send_shutdown()
                    except Exception:
                        pass
                    try:
                        self.control_uart.stop_link_maintenance()
                    except Exception:
                        pass
                    try:
                        self.control_uart.close()
                    except Exception:
                        pass
                    self.control_uart = None
                except Exception:
                    pass
        except Exception:
            pass

        # Wait until both serial connections are closed (or timeout), then close the app
        def _all_closed():
            uart_open = False
            user_open = False
            if self.control_uart:
                ser = getattr(self.control_uart, 'ser', None)
                uart_open = bool(ser and getattr(ser, 'is_open', False))
            if self.user_board:
                ser2 = getattr(self.user_board, 'ser', None)
                user_open = bool(ser2 and getattr(ser2, 'is_open', False))
            return (not uart_open) and (not user_open)

        timeout = 10.0
        interval = 0.5
        waited = 0.0
        while waited < timeout:
            if _all_closed():
                self.log_event("All serial connections closed. Exiting application.")
                try:
                    self._cancel_scheduled_callbacks()
                    self.master.destroy()
                except Exception:
                    pass
                return
            time.sleep(interval)
            waited += interval

        # Timeout expired, still closing to avoid hanging
        self.log_event("Timeout waiting for serial connections to close; exiting application anyway.")
        try:
            self._cancel_scheduled_callbacks()
            self.master.destroy()
        except Exception:
            pass


if __name__ == "__main__":
    root = Tk()
    app = AppUI(root)
    root.protocol("WM_DELETE_WINDOW", app.close)
    root.mainloop()
