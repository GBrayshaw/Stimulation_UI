from tkinter import Tk, Button, Checkbutton, IntVar, Label, StringVar, Text, Entry, END, DISABLED, NORMAL, messagebox
from datetime import datetime
from stim_io import UART_COMMS
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

        self.STOP_but = Button(master, text="STOP", command=self.STOP, width=10, height=2, state=DISABLED)
        self.STOP_but.grid(row=4, column=2, rowspan=3, padx=10, pady=10)

        # Reconnect button
        self.reconnect_but = Button(master, text="Reconnect", command=self.connect_control_board, state=NORMAL)
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
        # load_serial_numbers kept for compatibility but it won't read device_serials.txt
        self.connect_control_board()
        # self.initialise_user_board()

    def connect_control_board(self):
        if not self.control_board_serial:
            self.log_event("Control board serial port not set.")    # NOTE: Currently no way of setting in UI
            self.log_event("WARNING: NO CURRENT METHOD TO SET PORT IN UI. PLEASE EDIT THE CODE TO SET THE PORT.")
            self.disable_uart_controls()
            return
        else:
            try:
                self.control_uart = UART_COMMS(port=self.control_board_serial, baudrate=9600)
                self.log_event("Attempting handshake with control board...")
                
                for i in range(20):     # NOTE: Numbers currently arbraitrary; adjust as needed
                    if self.control_handshake():
                        break
                    self.log_event(f"Handshake {i} failed; retrying in 0.25s...")
                    time.sleep(0.25)
                    if i == 19:
                        raise ConnectionError("Handshake failed after multiple attempts.")

                self.log_event(f"Control board connected on port {self.control_board_serial}.")
                self.enable_uart_controls()
            except Exception as e:
                self.log_event(f"Failed to connect to control board: {e}")
                self.disable_uart_controls()

    def control_handshake(self):
        """Perform a handshake with the control board over UART."""
        if not self.control_uart:
            self.log_event("No control board currently connected for handshake.")
            return False
        try:
            self.control_uart.send_stim_ack()
            type, _, __ = self.control_uart.read()
            if type is UART_COMMS.MSG_TYPE.ACK:
                return True
            else:
                return False
        except Exception:
            return False

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
        if self.uart:
            self.uart.set_stim_amplitude(self.stim_amplitude)
            self.uart.set_pulse_width(self.pulse_width)

    def STOP(self):
        if self.uart:
            ack = self.uart.STOP()
        if ack:
            self.log_event("STIMULATION STOPPED")
        else:
            self.log_event("STOP ACK not received")

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

    def user_mode_ui(self):
        self.stim_up_but.config(state=NORMAL)
        self.stim_down_but.config(state=NORMAL)
        self.pulse_up_but.config(state=NORMAL)
        self.pulse_down_but.config(state=NORMAL)
        self.stim_amplitude_entry.config(state=NORMAL)
        self.pulse_width_entry.config(state=NORMAL)
        self.triggers_switch.config(state=NORMAL)
        self.recording_switch.config(state=NORMAL)

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
        if self.uart:
            self.STOP()
            self.uart.close()
        self.master.destroy()
    
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
            if self.uart:
                try:
                    # Ask control board to STOP and wait for ACK before closing comms
                    try:
                        self.log_event("Sending STOP to control board before shutdown...")
                        stopped = self.uart.STOP(attempts=5)
                    except Exception as e:
                        stopped = False
                        self.log_event(f"Error sending STOP to control board: {e}")

                    if stopped:
                        self.log_event("Control board acknowledged STOP.")
                    else:
                        self.log_event("Control board did not acknowledge STOP; proceeding to close comms.")

                    try:
                        self.uart.close()
                    except Exception:
                        pass
                    self.uart = None
                except Exception:
                    pass
        except Exception:
            pass

        # Wait until both serial connections are closed (or timeout), then close the app
        def _all_closed():
            uart_open = False
            user_open = False
            if self.uart:
                ser = getattr(self.uart, 'ser', None)
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
                    self.master.destroy()
                except Exception:
                    pass
                return
            time.sleep(interval)
            waited += interval

        # Timeout expired, still closing to avoid hanging
        self.log_event("Timeout waiting for serial connections to close; exiting application anyway.")
        try:
            self.master.destroy()
        except Exception:
            pass


if __name__ == "__main__":
    root = Tk()
    app = AppUI(root)
    root.protocol("WM_DELETE_WINDOW", app.close)
    root.mainloop()
