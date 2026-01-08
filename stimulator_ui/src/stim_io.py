import serial
import threading
import struct
import time
import uart_protocol
import queue


# Bind protocol constants from generated module
MODE = uart_protocol.MODE
PAYLOAD_TYPE = uart_protocol.PAYLOAD_TYPE
UART_START_BYTE = uart_protocol.UART_START_BYTE
UART_END_BYTE = uart_protocol.UART_END_BYTE
UART_MAX_PAYLOAD = uart_protocol.UART_MAX_PAYLOAD
HEARTBEAT_TIMEOUT_MS = uart_protocol.HEARTBEAT_TIMEOUT_MS

class UART_COMMS:
	"""Encapsulated UART communications using the framed protocol.

	Mirrors the behavior in comms.py: handshake with ACK, continuous heartbeat,
	and typed payload commands. Keeps a UI-compatible surface (write/readline/STOP).
	"""

	def __init__(self, port='/dev/ttyAMA0', baudrate=9600, heartbeat_f=20.0):
		self.port = port
		self.baudrate = baudrate
		self.comm_state = 1  # 0: User Mode, 1: PC Mode
		self.record = True
		# Heartbeat frequency: send at twice the timeout rate (2 / timeout_seconds)
		# i.e. for HEARTBEAT_TIMEOUT_MS = 300 ms, heartbeat_f ≈ 6.67 Hz
		self.heartbeat_f = 2000.0 / HEARTBEAT_TIMEOUT_MS
		self.last_ctrl_heartbeat = 0.0
		self.connected = False

		self._stop_event = threading.Event()
		self._write_lock = threading.Lock()
		self._line_buf = []  # synthesized text lines for UI compatibility
		self._hb_thread = None
		self._rx_thread = None
		self._events = queue.Queue()

		self.ser = serial.Serial(
			port=self.port,
			baudrate=self.baudrate,
			parity=serial.PARITY_NONE,
			stopbits=serial.STOPBITS_ONE,
			bytesize=serial.EIGHTBITS,
			timeout=0.5,
		)

	# ----------------------------
	# Framed protocol helpers
	# ----------------------------
	def _compute_crc(self, msg_type: int, payload_type: int, payload_bytes: bytes) -> int:
		crc = msg_type ^ payload_type ^ len(payload_bytes)
		for b in payload_bytes:
			crc ^= b
		return crc & 0xFF

	def _build_packet(self, msg_type: int, payload=None, payload_type: int = PAYLOAD_TYPE.PAYLOAD_NONE) -> bytes:
		if payload_type == PAYLOAD_TYPE.PAYLOAD_NONE:
			if payload not in (None, b''):
				raise ValueError('No payload expected for PAYLOAD_NONE')
			payload_bytes = b''
		elif payload_type == PAYLOAD_TYPE.PAYLOAD_INT32:
			if not isinstance(payload, int):
				raise TypeError('PAYLOAD_INT32 requires int payload')
			payload_bytes = payload.to_bytes(4, byteorder='big', signed=True)
		elif payload_type == PAYLOAD_TYPE.PAYLOAD_FLOAT32:
			if not isinstance(payload, (float, int)):
				raise TypeError('PAYLOAD_FLOAT32 requires float payload')
			payload_bytes = struct.pack('>f', float(payload))
		elif payload_type == PAYLOAD_TYPE.PAYLOAD_BYTES:
			if not isinstance(payload, (bytes, bytearray)):
				raise TypeError('PAYLOAD_BYTES requires bytes-like payload')
			payload_bytes = bytes(payload)
		else:
			raise ValueError('Unknown payload_type')

		if len(payload_bytes) > UART_MAX_PAYLOAD:
			raise ValueError('Payload too large')

		pkt = bytearray()
		pkt.append(UART_START_BYTE)
		pkt.append(msg_type)
		pkt.append(payload_type)
		pkt.append(len(payload_bytes))
		pkt.extend(payload_bytes)
		pkt.append(self._compute_crc(msg_type, payload_type, payload_bytes))
		pkt.append(UART_END_BYTE)
		return bytes(pkt)

	def _read_packet(self, timeout_s: float = 0.5):
		deadline = time.monotonic() + max(0.0, timeout_s)
		while time.monotonic() < deadline:
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

	def _send_packet(self, msg_type, payload=None, payload_type=PAYLOAD_TYPE.PAYLOAD_NONE):
		pkt = self._build_packet(msg_type, payload, payload_type)
		with self._write_lock:
			self.ser.write(pkt)

	# ----------------------------
	# Handshake and heartbeat
	# ----------------------------
	def handshake(self, overall_timeout_s=5.0, resend_interval_s=0.5) -> bool:
		ack_packet = self._build_packet(MODE.UART_MSG_ACK)
		start_time = time.monotonic()
		last_send = start_time

		while (time.monotonic() - start_time) < overall_timeout_s:
			pkt = self._read_packet(timeout_s=0.2)
			if pkt and pkt[0] == MODE.UART_MSG_ACK:
				# Echo ACK back to confirm link
				with self._write_lock:
					self.ser.write(ack_packet)

				self.connected = True
				# Start background heartbeat TX and RX processing
				self.start_link_maintenance()
				return True

			now = time.monotonic()
			if (now - last_send) >= resend_interval_s:
				with self._write_lock:
					self.ser.write(ack_packet)
				last_send = now

			time.sleep(0.05)

		return False

	def _heartbeat_loop(self):
		interval = 1.0 / self.heartbeat_f
		pkt = self._build_packet(MODE.UART_MSG_HEARTBEAT)
		while not self._stop_event.is_set():
			with self._write_lock:
				self.ser.write(pkt)
			time.sleep(interval)

	def _start_heartbeat(self):
		if self._hb_thread and self._hb_thread.is_alive():
			return
		self._hb_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
		self._hb_thread.start()

	def _rx_loop(self):
		while not self._stop_event.is_set():
			pkt = self._read_packet(timeout_s=0.2)
			if not pkt:
				continue
			msg_type, payload_type, payload = pkt
			self.last_ctrl_heartbeat = time.monotonic()
			# Parse shutdown message and enqueue event with source
			if msg_type == MODE.UART_MSG_SHUTDOWN:
				# If no payload, treat as receipt echo of our request
				if not payload or len(payload) == 0:
					try:
						self._events.put({"type": "receipt", "msg_type": MODE.UART_MSG_SHUTDOWN})
					except Exception:
						pass
					continue
				shutdown_src = None
				try:
					if payload_type == PAYLOAD_TYPE.PAYLOAD_INT32 and payload and len(payload) == 4:
						shutdown_src = int.from_bytes(payload, byteorder='big', signed=True)
					elif payload_type == PAYLOAD_TYPE.PAYLOAD_BYTES and payload:
						shutdown_src = int(payload[0])
				except Exception:
					shutdown_src = None
				try:
					self._events.put({"type": "shutdown", "source": shutdown_src})
				except Exception:
					pass
			# Receipts for echoed control requests (exclude heartbeat)
			elif msg_type in (
				MODE.UART_MSG_UNLOCK,
				MODE.UART_MSG_START_STIM,
				MODE.UART_MSG_GET_PARAMS,
				MODE.UART_MSG_UPDATE_WIDTH,
				MODE.UART_MSG_UPDATE_AMP,
			):
				try:
					self._events.put({"type": "receipt", "msg_type": msg_type})
				except Exception:
					pass
			# Parse status reports and enqueue status code
			elif msg_type == MODE.UART_MSG_STATUS:
				try:
					code = None
					if payload_type == PAYLOAD_TYPE.PAYLOAD_INT32 and payload and len(payload) == 4:
						code = int.from_bytes(payload, byteorder='big', signed=True)
					elif payload_type == PAYLOAD_TYPE.PAYLOAD_BYTES and payload:
						# accept first 4 bytes as big-endian int if provided
						b = bytes(payload[:4]).ljust(4, b"\x00")
						code = int.from_bytes(b, byteorder='big', signed=True)
					if code is not None:
						self._events.put({"type": "status", "code": code})
				except Exception:
					pass
			# Parse parameter reports from control board and enqueue updates
			elif msg_type == MODE.UART_MSG_SEND_WIDTH:
				try:
					width = None
					if payload_type == PAYLOAD_TYPE.PAYLOAD_INT32 and payload and len(payload) == 4:
						width = int.from_bytes(payload, byteorder='big', signed=True)
					elif payload_type == PAYLOAD_TYPE.PAYLOAD_BYTES and payload:
						# accept first 4 bytes when provided as raw
						b = bytes(payload[:4]).ljust(4, b"\x00")
						width = int.from_bytes(b, byteorder='big', signed=True)
					if width is not None:
						self._events.put({"type": "send_width", "width": width})
				except Exception:
					pass
			elif msg_type == MODE.UART_MSG_SEND_AMP:
				try:
					amp = None
					if payload_type == PAYLOAD_TYPE.PAYLOAD_FLOAT32 and payload and len(payload) == 4:
						amp = struct.unpack('>f', payload)[0]
					elif payload_type == PAYLOAD_TYPE.PAYLOAD_BYTES and payload and len(payload) >= 4:
						amp = struct.unpack('>f', bytes(payload[:4]))[0]
					if amp is not None:
						self._events.put({"type": "send_amp", "amp": float(amp)})
				except Exception:
					pass

	def _start_rx(self):
		if self._rx_thread and self._rx_thread.is_alive():
			return
		self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
		self._rx_thread.start()

	def start_link_maintenance(self):
		# Ensure stop flag is clear and start background TX/RX
		if self._stop_event.is_set():
			self._stop_event = threading.Event()
		self.last_ctrl_heartbeat = time.monotonic()
		self._start_heartbeat()
		self._start_rx()

	def stop_link_maintenance(self):
		# Stop background threads but keep serial open
		self._stop_event.set()
		try:
			if self._hb_thread:
				self._hb_thread.join(timeout=1.0)
		except Exception:
			pass
		try:
			if self._rx_thread:
				self._rx_thread.join(timeout=1.0)
		except Exception:
			pass
		self._hb_thread = None
		self._rx_thread = None

	# TODO: Logic for recieving data
	def read(self, timeout_s: float = 0.5):
		res = self._read_packet(timeout_s=timeout_s)
		if not res:
			return None, None, None
		msg_type, payload_type, payload = res
		self.last_ctrl_heartbeat = time.monotonic()
		return msg_type, payload_type, payload

	def toggle_PC_usr(self):
		self.comm_state ^= 1
		return True

	def toggle_recording(self):
		if self.record:
			self._send_packet(MODE.UART_MSG_STIM)
			self.record = False
		else:
			self._send_packet(MODE.UART_MSG_RECORD)
			self.record = True

	def set_mode_stim(self):
		"""Explicitly set system mode to STIM on the control board."""
		self._send_packet(MODE.UART_MSG_STIM)
		self.record = False

	def set_mode_record(self):
		"""Explicitly set system mode to RECORD on the control board."""
		self._send_packet(MODE.UART_MSG_RECORD)
		self.record = True

	def send_shutdown(self, source: int | None = None):
		# Default to SHUTDOWN_SRC_PI when source not provided
		try:
			src = source if source is not None else getattr(uart_protocol.SHUTDOWN_SRC, 'SHUTDOWN_SRC_PI', 3)
		except Exception:
			src = 3
		self._send_packet(MODE.UART_MSG_SHUTDOWN, int(src), PAYLOAD_TYPE.PAYLOAD_INT32)

	def send_unlock(self):
		# Send a START command (no payload)
		self._send_packet(MODE.UART_MSG_UNLOCK)

	def send_get_params(self):
		# Request current parameters from control board
		self._send_packet(MODE.UART_MSG_GET_PARAMS)

	def send_start_stim(self):
		# Send START_STIM command (no payload)
		self._send_packet(MODE.UART_MSG_START_STIM)

	def set_stim_amplitude(self, amplitude):
		self._send_packet(MODE.UART_MSG_UPDATE_AMP, float(amplitude), PAYLOAD_TYPE.PAYLOAD_FLOAT32)

	def set_pulse_width(self, width):
		self._send_packet(MODE.UART_MSG_UPDATE_WIDTH, int(width), PAYLOAD_TYPE.PAYLOAD_INT32)

	def is_link_alive(self, timeout_ms: int | None = None) -> bool:
		if self.last_ctrl_heartbeat <= 0:
			return False
		limit_ms = HEARTBEAT_TIMEOUT_MS if timeout_ms is None else timeout_ms
		return (time.monotonic() - self.last_ctrl_heartbeat) < (limit_ms / 1000.0)

	def get_events(self, max_items: int = 10):
		items = []
		try:
			for _ in range(max_items):
				items.append(self._events.get_nowait())
		except Exception:
			pass
		return items

	def close(self):
		self.stop_link_maintenance()
		try:
			self.ser.close()
		except Exception:
			pass
