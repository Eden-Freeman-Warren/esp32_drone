from __future__ import annotations

import queue
import re
import threading
import time
from typing import Optional

import cflib.crtp
import serial
from cflib.crtp.crtpdriver import CRTPDriver
from cflib.crtp.crtpstack import CRTPPacket
from cflib.crtp.exceptions import WrongUriType


SYNC_BYTE = 0xAA
MAX_FRAME_PAYLOAD = 64
MIN_TX_GAP_SECONDS = 0.003

# Opening a USB serial port can reset or reinitialize the XIAO.
# Wait before cflib sends its first protocol request.
BRIDGE_STARTUP_WAIT_SECONDS = 3.0

# Prevent SyncCrazyflie from waiting forever when no reply arrives.
FIRST_REPLY_TIMEOUT_SECONDS = 12.0


class EspNowDriver(CRTPDriver):
    _URI_PATTERN = re.compile(
        r"^espnow://(?P<port>[^/]+)/(?P<baud>[0-9]+)$"
    )

    def __init__(self) -> None:
        super().__init__()

        self.needs_resending = True

        self._serial: Optional[serial.Serial] = None
        self._rx_queue: queue.Queue[CRTPPacket] = queue.Queue()

        self._reader_thread: Optional[threading.Thread] = None
        self._watchdog_thread: Optional[threading.Thread] = None

        self._stop_event = threading.Event()
        self._first_reply_event = threading.Event()

        self._write_lock = threading.Lock()
        self._last_write_time = 0.0
        self._link_error_callback = None
        self._error_reported = threading.Event()

    def connect(
        self,
        uri,
        radio_link_statistics_callback,
        link_error_callback,
    ) -> None:
        match = self._URI_PATTERN.fullmatch(uri)
        if match is None:
            raise WrongUriType("Not an ESP-NOW URI")

        del radio_link_statistics_callback

        port = match.group("port")
        baud = int(match.group("baud"))

        self._link_error_callback = link_error_callback
        self._stop_event.clear()
        self._first_reply_event.clear()
        self._error_reported.clear()

        while not self._rx_queue.empty():
            try:
                self._rx_queue.get_nowait()
            except queue.Empty:
                break

        self._serial = serial.Serial(
            port=port,
            baudrate=baud,
            timeout=0.05,
            write_timeout=1.0,
            rtscts=False,
            dsrdtr=False,
        )

        # Give the XIAO time to finish setup if opening COM caused a reset.
        time.sleep(BRIDGE_STARTUP_WAIT_SECONDS)

        self._serial.reset_input_buffer()
        self._serial.reset_output_buffer()

        self._reader_thread = threading.Thread(
            target=self._reader_loop,
            name="EspNowDriverReader",
            daemon=True,
        )
        self._reader_thread.start()

        self._watchdog_thread = threading.Thread(
            target=self._first_reply_watchdog,
            name="EspNowDriverWatchdog",
            daemon=True,
        )
        self._watchdog_thread.start()

    def send_packet(self, packet: CRTPPacket) -> None:
        ser = self._serial
        if ser is None or not ser.is_open:
            raise RuntimeError("ESP-NOW serial link is not open.")

        header = packet.get_header() & 0xF3
        crtp = bytes([header]) + bytes(packet.data)

        payload = crtp + bytes([sum(crtp) & 0xFF])

        if len(payload) > MAX_FRAME_PAYLOAD:
            raise ValueError(
                f"CRTP frame is too large: {len(payload)} bytes"
            )

        frame = bytes([SYNC_BYTE, len(payload)]) + payload

        try:
            with self._write_lock:
                delay = (
                    self._last_write_time
                    + MIN_TX_GAP_SECONDS
                    - time.monotonic()
                )
                if delay > 0:
                    time.sleep(delay)

                written = ser.write(frame)
                if written != len(frame):
                    raise serial.SerialTimeoutException(
                        f"Wrote {written}/{len(frame)} bytes."
                    )

                self._last_write_time = time.monotonic()

        except Exception as error:
            self._report_link_error(
                f"ESP-NOW serial send failed: {error}"
            )

    def receive_packet(self, wait: float = 0):
        try:
            if wait < 0:
                return self._rx_queue.get()
            if wait == 0:
                return self._rx_queue.get_nowait()
            return self._rx_queue.get(timeout=wait)
        except queue.Empty:
            return None

    def close(self) -> None:
        self._stop_event.set()
        current = threading.current_thread()

        if (
            self._reader_thread
            and self._reader_thread.is_alive()
            and self._reader_thread is not current
        ):
            self._reader_thread.join(timeout=1.0)

        if (
            self._watchdog_thread
            and self._watchdog_thread.is_alive()
            and self._watchdog_thread is not current
        ):
            self._watchdog_thread.join(timeout=1.0)

        if self._serial is not None and self._serial.is_open:
            try:
                self._serial.close()
            except serial.SerialException:
                pass

        self._serial = None
        self._reader_thread = None
        self._watchdog_thread = None
        self._link_error_callback = None

    def get_name(self) -> str:
        return "espnow"

    def get_status(self) -> str:
        if self._serial is not None and self._serial.is_open:
            return "ESP-NOW serial link open"
        return "ESP-NOW serial link closed"

    def scan_interface(self, address=None):
        del address
        return []

    def get_help(self) -> str:
        return "URI format: espnow://COM9/115200"

    def _reader_loop(self) -> None:
        try:
            while not self._stop_event.is_set():
                sync = self._read_exact(1)
                if sync is None:
                    continue

                if sync[0] != SYNC_BYTE:
                    continue

                length_bytes = self._read_exact(1)
                if length_bytes is None:
                    continue

                payload_length = length_bytes[0]
                if not 2 <= payload_length <= MAX_FRAME_PAYLOAD:
                    continue

                payload = self._read_exact(payload_length)
                if payload is None:
                    continue

                crtp = payload[:-1]
                received_checksum = payload[-1]

                if (sum(crtp) & 0xFF) != received_checksum:
                    continue

                self._first_reply_event.set()

                self._rx_queue.put(
                    CRTPPacket(
                        header=crtp[0],
                        data=crtp[1:],
                    )
                )

        except Exception as error:
            if not self._stop_event.is_set():
                self._report_link_error(
                    f"ESP-NOW serial receive failed: {error}"
                )

    def _first_reply_watchdog(self) -> None:
        received = self._first_reply_event.wait(
            FIRST_REPLY_TIMEOUT_SECONDS
        )

        if not received and not self._stop_event.is_set():
            self._report_link_error(
                "No CRTP reply received within "
                f"{FIRST_REPLY_TIMEOUT_SECONDS:.0f} seconds. "
                "Power-cycle the XIAO and drone, then retry."
            )

    def _read_exact(self, byte_count: int) -> Optional[bytes]:
        ser = self._serial
        if ser is None:
            return None

        result = bytearray()

        while (
            len(result) < byte_count
            and not self._stop_event.is_set()
        ):
            chunk = ser.read(byte_count - len(result))

            if not chunk:
                return None

            result.extend(chunk)

        if len(result) != byte_count:
            return None

        return bytes(result)

    def _report_link_error(self, message: str) -> None:
        # Only report the first fatal error. cflib may close the driver
        # synchronously from this callback.
        if self._error_reported.is_set():
            return

        self._error_reported.set()
        callback = self._link_error_callback

        if callback is not None:
            callback(message)


def register_espnow_driver() -> None:
    cflib.crtp.init_drivers()

    if EspNowDriver not in cflib.crtp.CLASSES:
        cflib.crtp.CLASSES.insert(0, EspNowDriver)