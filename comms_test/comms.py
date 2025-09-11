import serial
import time
import uart_protocol
import struct
import threading
import sys

MODE = uart_protocol.MODE
PAYLOAD_TYPE = uart_protocol.PAYLOAD_TYPE

UART_START_BYTE = uart_protocol.UART_START_BYTE
UART_END_BYTE = uart_protocol.UART_END_BYTE
UART_MAX_PAYLOAD = uart_protocol.UART_MAX_PAYLOAD

HEARTBEAT_TIMEOUT_MS = uart_protocol.HEARTBEAT_TIMEOUT_MS
HEARTBEAT_FREQ_MULTIPLIER = 1.5
HEARTBEAT_FREQ = 1 / (HEARTBEAT_TIMEOUT_MS / 1000.0 * HEARTBEAT_FREQ_MULTIPLIER)  # Send at HEARTBEAT_FREQ_MULTIPLIERx the timeout rate

# ----------------------------
# CRC (must match Arduino)
# ----------------------------
def compute_crc(msg_type, payload_type, payload):
    crc = msg_type ^ payload_type ^ len(payload)
    for b in payload:
        crc ^= b
    return crc & 0xFF

# ----------------------------
# Packet builder (TX)
# ----------------------------
def build_packet(msg_type, payload=None, payload_type=PAYLOAD_TYPE.PAYLOAD_NONE):
    """
    Build a UART packet according to protocol.

    payload_type determines how payload is interpreted:
      - PAYLOAD_NONE: payload must be None
      - PAYLOAD_INT32: payload must be int, 4 bytes big-endian
      - PAYLOAD_FLOAT32: payload must be float, 4 bytes IEEE-754 big-endian
      - PAYLOAD_BYTES: payload must be bytes-like
    """

    # ------------------------
    # Encode payload to bytes
    # ------------------------
    if payload_type == PAYLOAD_TYPE.PAYLOAD_NONE:
        if payload not in (None, b''):
            raise ValueError("No payload expected for PAYLOAD_NONE")
        payload_bytes = b''

    elif payload_type == PAYLOAD_TYPE.PAYLOAD_INT32:
        if not isinstance(payload, int):
            raise TypeError("PAYLOAD_INT32 requires int payload")
        payload_bytes = payload.to_bytes(4, byteorder='big', signed=True)

    elif payload_type == PAYLOAD_TYPE.PAYLOAD_FLOAT32:
        if not isinstance(payload, float):
            raise TypeError("PAYLOAD_FLOAT32 requires float payload")
        payload_bytes = struct.pack('>f', payload)  # big-endian float

    elif payload_type == PAYLOAD_TYPE.PAYLOAD_BYTES:
        if not isinstance(payload, (bytes, bytearray)):
            raise TypeError("PAYLOAD_BYTES requires bytes-like payload")
        payload_bytes = bytes(payload)

    else:
        raise ValueError(f"Unknown payload_type: {payload_type}")

    length = len(payload_bytes)
    if length > UART_MAX_PAYLOAD:
        raise ValueError("Payload too large")

    # ------------------------
    # Build the packet
    # ------------------------
    packet = bytearray()
    packet.append(UART_START_BYTE)
    packet.append(msg_type)
    packet.append(payload_type)
    packet.append(length)
    packet.extend(payload_bytes)
    packet.append(compute_crc(msg_type, payload_type, payload_bytes))
    packet.append(UART_END_BYTE)

    return bytes(packet)



# ----------------------------
# Packet parser (RX)
# ----------------------------
def read_packet(ser):
    """
    Reads and validates a single UART packet.
    Returns msg_type on success, or None on failure/timeout.
    """

    # 1. Wait for START byte
    start = ser.read(1)
    if not start or start[0] != UART_START_BYTE:
        return None

    # 2. Read fixed header
    header = ser.read(3)
    if len(header) != 3:
        return None

    msg_type, payload_type, length = header

    # No payloads should ever be sent to the Pi
    if length != 0:
        # Discard payload + CRC + END
        ser.read(length + 2)
        return None

    # 3. Read CRC + END
    tail = ser.read(2)
    if len(tail) != 2:
        return None

    crc_rx, end = tail

    if end != UART_END_BYTE:
        return None

    # 4. Validate CRC
    crc_calc = compute_crc(msg_type, payload_type, b"")
    if crc_rx != crc_calc:
        return None

    return msg_type


def handshake_with_arduino(
    ser,
    overall_timeout_s=5.0,
    resend_interval_s=0.5
    ):
    """
    Perform ACK handshake with Arduino.

    Returns True on success, False on timeout.
    """

    ack_packet = build_packet(MODE.UART_MSG_ACK)

    start_time = time.monotonic()
    last_send = start_time
    
    while (time.monotonic() - start_time) < overall_timeout_s:
        # Check for incoming packet
        msg_type = read_packet(ser)

        if msg_type == MODE.UART_MSG_ACK:
            # Echo ACK back to confirm link
            ser.write(ack_packet)
            print("Arduino ACK received and echoed.")
            return True

        # Resend ACK periodically
        now = time.monotonic()
        if (now - last_send) >= resend_interval_s:
            ser.write(ack_packet)
            last_send = now

        time.sleep(0.05)

    print("Timeout waiting for Arduino ACK")
    return False


def heartbeat_loop(ser, stop_event, write_lock, hz: float = HEARTBEAT_FREQ):
    interval = 1.0 / hz
    packet = build_packet(MODE.UART_MSG_HEARTBEAT)
    while not stop_event.is_set():
        with write_lock:
            ser.write(packet)
        # Keep logs lightweight to avoid spamming
        # print("Sent HEARTBEAT")
        # Sleep precise-ish; adjust if timing drift is critical
        time.sleep(interval)


def input_loop(ser, stop_event, write_lock):
    print("Enter pulse width (int), or 'q' to quit:")
    while not stop_event.is_set():
        try:
            # Non-thread-safe input will block this thread only
            line = sys.stdin.readline()
            if not line:
                # EOF (e.g., piped input closed)
                stop_event.set()
                break
            line = line.strip()
            if line.lower() in ("q", "quit", "exit"):
                stop_event.set()
                break
            try:
                width = int(line)
            except ValueError:
                print("Invalid width. Enter an integer or 'q' to quit.")
                continue
            packet = build_packet(MODE.UART_MSG_UPDATE_WIDTH, width, PAYLOAD_TYPE.PAYLOAD_INT32)
            with write_lock:
                ser.write(packet)
            print(f"Sent {width} pulse width to ctrl board")
        except Exception as e:
            print(f"Input error: {e}")
            # Continue listening unless fatal
            time.sleep(0.1)


def checkheartbeat(last_beat):
    time_elapsed = time.monotonic() - last_beat
    
    if time_elapsed > 5.0:
        return False
    else:
        return True


# ----------------------------
# Main
# ----------------------------
def main():
    ser = serial.Serial(
        port="/dev/ttyAMA0",
        baudrate=9600,
        timeout=0.5
    )

    ctrl_heartbeat = time.monotonic()

    print("Waiting for ctrl board handshake...")
    while not handshake_with_arduino(ser):
        time.sleep(0.05)

    print("Connected to Control Board. Ctrl-C to exit.\n")

    stop_event = threading.Event()
    write_lock = threading.Lock()

    hb_thread = threading.Thread(target=heartbeat_loop, args=(ser, stop_event, write_lock), kwargs={"hz": 20.0}, daemon=True)
    in_thread = threading.Thread(target=input_loop, args=(ser, stop_event, write_lock), daemon=True)

    hb_thread.start()
    in_thread.start()

    try:
        # Serial RX/monitor loop
        while not stop_event.is_set():

            active = checkheartbeat(ctrl_heartbeat)

            msg_type = read_packet(ser)
            if msg_type is None:
                # No valid packet received in timeout; keep looping
                continue
            else:
                ctrl_heartbeat = time.monotonic()
                if msg_type == MODE.UART_MSG_ACK:
                    print("Received ACK")
                elif msg_type == MODE.UART_MSG_ERROR:
                    print("Received ERROR")
                else:
                    # print(f"Received message type: 0x{msg_type:02X}")
                    pass

    except KeyboardInterrupt:
        print("\nExiting...")
        stop_event.set()
    finally:
        stop_event.set()
        hb_thread.join(timeout=1.0)
        in_thread.join(timeout=1.0)
        ser.close()


if __name__ == "__main__":
    main()
