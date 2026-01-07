from tkinter import Tk, Button, Checkbutton, IntVar, Label, StringVar, Text, Entry, END, DISABLED, NORMAL, messagebox, Frame
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
        self.pending_stim_amplitude = 0.00
        self.pending_pulse_width = 0.00
        self.nerve_impedance = 0.00

        # Display Labels
        self.stim_amplitude_var = StringVar()
        self.stim_amplitude_var.set(f"Stim Amplitude: {self.stim_amplitude:.2f}uA")
        self.stim_amplitude_label = Label(master, textvariable=self.stim_amplitude_var)
        self.stim_amplitude_label.grid(row=0, column=0, columnspan=2, padx=10, pady=10)

        self.pulse_width_var = StringVar()
        self.pulse_width_var.set(f"Pulse Width: {self.pulse_width:.2f}")
        self.pulse_width_label = Label(master, textvariable=self.pulse_width_var)
        self.pulse_width_label.grid(row=2, column=0, columnspan=2, padx=10, pady=10)

        self.nerve_impedance_var = StringVar()
        self.nerve_impedance_var.set(f"Nerve Impedance: {self.nerve_impedance:.2f}ohms")
        self.nerve_impedance_label = Label(master, textvariable=self.nerve_impedance_var)
        self.nerve_impedance_label.grid(row=2, column=2, padx=10, pady=10)

        # UI Buttons
        self.stim_up_but = Button(master, text="+", command=self.stim_amp_up, width=10, height=2, state=DISABLED)
        self.stim_up_but.grid(row=1, column=0, padx=10, pady=10)

        self.stim_down_but = Button(master, text="-", command=self.stim_amp_down, width=10, height=2, state=DISABLED)
        self.stim_down_but.grid(row=1, column=1, padx=10, pady=10)

        self.pulse_up_but = Button(master, text="+", command=self.pulse_width_up, width=10, height=2, state=DISABLED)
        self.pulse_up_but.grid(row=3, column=0, padx=10, pady=10)

        self.pulse_down_but = Button(master, text="-", command=self.pulse_width_down, width=10, height=2, state=DISABLED)
        self.pulse_down_but.grid(row=3, column=1, padx=10, pady=10)

        self.done_but = Button(master, text="Done", command=self.apply_settings, width=10, height=2)
        self.done_but.grid(row=4, column=0, columnspan=2, padx=10, pady=10)

        # Control actions: START above STOP in a stacked frame
        self.ctrl_actions = Frame(master)
        self.ctrl_actions.grid(row=4, column=2, padx=10, pady=10, sticky="n")
        self.start_but = Button(self.ctrl_actions, text="START", command=self.start_system, width=10, height=2, state=DISABLED)
        self.start_but.pack(padx=5, pady=(0,5))
        self.STOP_but = Button(self.ctrl_actions, text="STOP", command=self.STOP, width=10, height=2, state=DISABLED)
        self.STOP_but.pack(padx=5, pady=5)

        # Reconnect button
        self.reconnect_but = Button(master, text="Reconnect", command=self.start_auto_connect, state=NORMAL)
        self.reconnect_but.grid(row=6, column=2, rowspan=3, padx=10, pady=10)

        # Reconnect User Board button
        # self.reconnect_user_but = Button(master, text="Reconnect User Board", command=self.initialise_user_board, state=DISABLED)
        # self.reconnect_user_but.grid(row=7, column=2, rowspan=3, padx=10, pady=10)

        # Poll Status button
        self.poll_status_but = Button(master, text="Poll Status", command=self.poll_status, width=10, height=2, state=DISABLED)
        self.poll_status_but.grid(row=9, column=2, padx=10, pady=10)

        # Entry boxes for manual input
        self.stim_amplitude_entry = Entry(master, state=DISABLED)
        self.stim_amplitude_entry.grid(row=1, column=2, padx=10, pady=10)
        self.stim_amplitude_entry.bind("<Return>", self.update_stim_amplitude)

        self.pulse_width_entry = Entry(master, state=DISABLED)
        self.pulse_width_entry.grid(row=3, column=2, padx=10, pady=10)
        self.pulse_width_entry.bind("<Return>", self.update_pulse_width)

        # Switches
        self.triggers_label = Label(master, text="Triggers (Internal - External)")
        self.triggers_label.grid(row=5, column=0, columnspan=2, padx=10, pady=10)

        self.switch_var = IntVar()
        self.triggers_switch = Checkbutton(master, text="", variable=self.switch_var, command=self.toggle_trigger, state=DISABLED)
        self.triggers_switch.grid(row=6, column=0, columnspan=2, padx=10, pady=0)

        self.recording_switch_var = IntVar()
        self.recording_switch = Checkbutton(master, text="Recording - Stimulation", variable=self.recording_switch_var, command=self.toggle_recording, state=DISABLED)
        self.recording_switch.grid(row=7, column=0, columnspan=2, padx=10, pady=10)

        self.pc_user_switch_var = IntVar()
        self.pc_user_switch = Checkbutton(master, text="PC - User", variable=self.pc_user_switch_var, command=self.toggle_pc_user, state=DISABLED)
        self.pc_user_switch.grid(row=8, column=0, columnspan=2, padx=10, pady=10)

        # Event log
        self.event_log = Text(master, width=80, height=20)
        self.event_log.grid(row=0, column=3, rowspan=14, padx=10, pady=10)

        # Shutdown button (upper-right)
        self.shutdown_but = Button(master, text="Shutdown", command=self.shutdown_system, width=12, height=2)
        self.shutdown_but.grid(row=0, column=4, padx=10, pady=10)

        # Initialize serial communication
        # We no longer use saved serial numbers. Control board is on a fixed UART port,
        # the user board is expected to be the only connected USB serial device.
        self.control_board_serial = "/dev/ttyAMA0"
        self.user_board_serial = None
        self.control_uart = None
        self.user_board = None

        self.pc_usr_toggle = 1
        # Heartbeat monitoring
        self._hb_timeout_ms = getattr(uart_protocol, 'HEARTBEAT_TIMEOUT_MS', 300)
        self._hb_timeout_multiplier = 1.5  # Add margin to avoid false positives at startup
        self._hb_check_interval_ms = max(50, int(self._hb_timeout_ms / 2))
        self._hb_monitor_id = None
        self._hb_warmup_until = None  # grace window after connect
        self._hb_tripped = False

        # Connection attempt state (non-blocking auto-connect)
        self._connect_after_id = None
        self._connect_retry_ms = 250
        self._connect_attempts = 0

        # Start UI immediately and begin seeking the control board
        self.log_event(f"Seeking control board on {self.control_board_serial}...")
        self.start_auto_connect()
        # self.initialise_user_board()

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
                # Start heartbeat monitor with warmup
                self._hb_warmup_until = time.monotonic() + 3.0
                self._schedule_heartbeat_monitor()
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
                    try:
                        if src == getattr(uart_protocol.SHUTDOWN_SRC, 'SHUTDOWN_SRC_PI', 3):
                            origin = "Raspberry Pi"
                        elif src == getattr(uart_protocol.SHUTDOWN_SRC, 'SHUTDOWN_SRC_CONTROL', 1):
                            origin = "Control Board"
                        elif src == getattr(uart_protocol.SHUTDOWN_SRC, 'SHUTDOWN_SRC_STIM', 2):
                            origin = "Stimulator Board"
                    except Exception:
                        pass
                    self.log_event(f"Shutdown message received from: {origin}")
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

    def stim_amp_up(self):
        self.pending_stim_amplitude += 1.00
        self.stim_amplitude_var.set(f"Stim Amplitude: {self.pending_stim_amplitude:.2f}uA")

    def stim_amp_down(self):
        self.pending_stim_amplitude -= 1.00
        self.stim_amplitude_var.set(f"Stim Amplitude: {self.pending_stim_amplitude:.2f}uA")

    def pulse_width_up(self):
        self.pending_pulse_width += 1.00
        self.pulse_width_var.set(f"Pulse Width: {self.pending_pulse_width:.2f}")

    def pulse_width_down(self):
        self.pending_pulse_width -= 1.00
        if self.pending_pulse_width < 0:
            self.pending_pulse_width = 0
        self.pulse_width_var.set(f"Pulse Width: {self.pending_pulse_width:.2f}")

    def apply_settings(self):
        """Apply the pending stimulation amplitude and pulse width settings."""
        self.stim_amplitude = self.pending_stim_amplitude
        self.pulse_width = self.pending_pulse_width
        self.log_event(f"Applied settings: Stim Amplitude = {self.stim_amplitude:.2f}uA, Pulse Width = {self.pulse_width:.2f}")
        if not self.control_uart:
            self.log_event("Cannot apply settings: control board not connected.")
            return
        try:
            # Send width (int) and amplitude (float) to control board
            self.control_uart.set_pulse_width(self.pulse_width)
            self.control_uart.set_stim_amplitude(self.stim_amplitude)
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
        if self.uart:
            self.uart.toggle_trigger()
        if self.switch_var.get():
            self.log_event("INTERNAL TRIGGERS")
        else:
            self.log_event("EXTERNAL TRIGGERS")

    def toggle_recording(self):
        if self.uart:
            self.uart.toggle_recording()
        if self.recording_switch_var.get():
            self.log_event("Recording Mode Enabled")
            self.recording_ui()
        else:
            self.log_event("Stimulation Mode Enabled")
            self.stimulation_ui()

    def toggle_pc_user(self):
        if not self.user_board:
            self.log_event("Cannot toggle PC/User mode: User board not connected")
            return
        if self.uart:
            ack = self.uart.toggle_PC_usr()
        if ack:
            self.pc_usr_toggle ^= 1
            if self.pc_user_switch_var.get():
                self.log_event("PC Mode Enabled")
                self.pc_mode_ui()
            else:
                self.log_event("User Mode Enabled")
                self.user_mode_ui()
        else:
            self.log_event("User Toggle ACK not received")

    def recording_ui(self):
        self.stim_up_but.config(state=DISABLED)
        self.stim_down_but.config(state=DISABLED)
        self.pulse_up_but.config(state=DISABLED)
        self.pulse_down_but.config(state=DISABLED)
        self.stim_amplitude_entry.config(state=DISABLED)
        self.pulse_width_entry.config(state=DISABLED)
        self.triggers_switch.config(state=DISABLED)
        self.pc_user_switch.config(state=DISABLED)
        self.done_but.config(state=DISABLED)

    def stimulation_ui(self):
        self.stim_up_but.config(state=NORMAL)
        self.stim_down_but.config(state=NORMAL)
        self.pulse_up_but.config(state=NORMAL)
        self.pulse_down_but.config(state=NORMAL)
        self.stim_amplitude_entry.config(state=NORMAL)
        self.pulse_width_entry.config(state=NORMAL)
        self.triggers_switch.config(state=NORMAL)
        self.pc_user_switch.config(state=NORMAL)
        self.poll_status_but.config(state=NORMAL)
        self.start_but.config(state=NORMAL)

    def pc_mode_ui(self):
        self.stim_up_but.config(state=DISABLED)
        self.stim_down_but.config(state=DISABLED)
        self.pulse_up_but.config(state=DISABLED)
        self.pulse_down_but.config(state=DISABLED)
        self.stim_amplitude_entry.config(state=DISABLED)
        self.pulse_width_entry.config(state=DISABLED)
        self.triggers_switch.config(state=DISABLED)
        self.recording_switch.config(state=DISABLED)
        self.poll_status_but.config(state=DISABLED)
        self.start_but.config(state=DISABLED)

    def user_mode_ui(self):
        self.stim_up_but.config(state=NORMAL)
        self.stim_down_but.config(state=NORMAL)
        self.pulse_up_but.config(state=NORMAL)
        self.pulse_down_but.config(state=NORMAL)
        self.stim_amplitude_entry.config(state=NORMAL)
        self.pulse_width_entry.config(state=NORMAL)
        self.triggers_switch.config(state=NORMAL)
        self.recording_switch.config(state=NORMAL)
        self.start_but.config(state=NORMAL)

    def update_stim_amplitude(self, event):
        """Update the pending stimulation amplitude from the entry box."""
        try:
            value = float(self.stim_amplitude_entry.get())
            self.pending_stim_amplitude = value
            self.stim_amplitude_var.set(f"Stim Amplitude: {self.pending_stim_amplitude:.2f}uA")
        except ValueError:
            self.log_event("Invalid input for Stimulation Amplitude")

    def update_pulse_width(self, event):
        """Update the pending pulse width from the entry box."""
        try:
            value = float(self.pulse_width_entry.get())
            self.pending_pulse_width = value
            self.pulse_width_var.set(f"Pulse Width: {self.pending_pulse_width:.2f}")
        except ValueError:
            self.log_event("Invalid input for Pulse Width")

    def disable_ui(self):
        self.stim_up_but.config(state=DISABLED)
        self.stim_down_but.config(state=DISABLED)
        self.pulse_up_but.config(state=DISABLED)
        self.pulse_down_but.config(state=DISABLED)
        self.stim_amplitude_entry.config(state=DISABLED)
        self.pulse_width_entry.config(state=DISABLED)
        self.triggers_switch.config(state=DISABLED)
        self.recording_switch.config(state=DISABLED)
        self.pc_user_switch.config(state=DISABLED)
        self.STOP_but.config(state=DISABLED)
        self.done_but.config(state=DISABLED)

    def disable_uart_controls(self):
        """Disable only the controls that require the control board (UART)."""
        self.stim_up_but.config(state=DISABLED)
        self.stim_down_but.config(state=DISABLED)
        self.pulse_up_but.config(state=DISABLED)
        self.pulse_down_but.config(state=DISABLED)
        self.stim_amplitude_entry.config(state=DISABLED)
        self.pulse_width_entry.config(state=DISABLED)
        self.triggers_switch.config(state=DISABLED)
        self.recording_switch.config(state=DISABLED)
        # PC/User switch depends on user board; keep it as-is if user board present
        self.pc_user_switch.config(state=DISABLED)
        self.STOP_but.config(state=DISABLED)
        self.done_but.config(state=DISABLED)
        self.poll_status_but.config(state=DISABLED)
        self.start_but.config(state=DISABLED)

    def enable_uart_controls(self):
        """Enable only the controls that require the control board (UART)."""
        self.stim_up_but.config(state=NORMAL)
        self.stim_down_but.config(state=NORMAL)
        self.pulse_up_but.config(state=NORMAL)
        self.pulse_down_but.config(state=NORMAL)
        self.stim_amplitude_entry.config(state=NORMAL)
        self.pulse_width_entry.config(state=NORMAL)
        self.triggers_switch.config(state=NORMAL)
        self.recording_switch.config(state=NORMAL)
        # Only enable PC/User switch if user board is present
        if getattr(self, 'user_board', None):
            self.pc_user_switch.config(state=NORMAL)
        else:
            self.pc_user_switch.config(state=DISABLED)
        self.STOP_but.config(state=NORMAL)
        self.done_but.config(state=NORMAL)
        self.poll_status_but.config(state=NORMAL)
        self.start_but.config(state=NORMAL)

    def start_system(self):
        if not self.control_uart:
            self.log_event("Cannot START: control board not connected.")
            return
        try:
            self.control_uart.send_start()
            self.log_event("START: sent START to control board.")
        except Exception as e:
            self.log_event(f"START: failed to send START: {e}")

    def enable_ui(self):
        self.stim_up_but.config(state=NORMAL)
        self.stim_down_but.config(state=NORMAL)
        self.pulse_up_but.config(state=NORMAL)
        self.pulse_down_but.config(state=NORMAL)
        self.stim_amplitude_entry.config(state=NORMAL)
        self.pulse_width_entry.config(state=NORMAL)
        self.triggers_switch.config(state=NORMAL)
        self.recording_switch.config(state=NORMAL)
        self.pc_user_switch.config(state=NORMAL)
        self.STOP_but.config(state=NORMAL)
        self.done_but.config(state=NORMAL)
        self.poll_status_but.config(state=NORMAL)

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
    
    def poll_status(self):
        """Send an <ACK> message to the serial device and log the response."""
        if self.uart:
            raw = self.uart_handshake()
            if raw is None:
                self.log_event("No response to <ACK>; UART comms appear disconnected.")
                self.handle_uart_disconnect("no response to <ACK>")
                return
            try:
                response = raw.decode('utf-8').strip()
            except Exception:
                response = repr(raw)
            self.log_event(f"Poll Status Response: {response}")
        else:
            self.log_event("Cannot poll status: UART comms not connected.")

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
        try:
            if self.uart:
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
