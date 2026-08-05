"""Raw Modbus RTU for A6-400RS — ported from the proven /home/pi/ballscrew/a6.py client."""

from __future__ import annotations

import logging
import threading
import time
from typing import Sequence

import serial

logger = logging.getLogger(__name__)


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc & 0xFFFF


class ModbusException(RuntimeError):
    pass


class ModbusBus:
    """Thread-safe raw Modbus RTU bus (8E1 by default), multi-slave via unit id."""

    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        *,
        parity: str = "E",
        stopbits: int = 1,
        bytesize: int = 8,
        timeout: float = 0.5,
        inter_frame_s: float = 0.02,
        handle_local_echo: bool = False,  # unused; kept for call-site compat
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.parity = (parity or "E").upper()
        self.stopbits = stopbits
        self.bytesize = bytesize
        self.timeout = timeout
        self.inter_frame_s = inter_frame_s
        self._ser: serial.Serial | None = None
        self._lock = threading.RLock()
        self._connected = False

    @property
    def connected(self) -> bool:
        return self._connected and self._ser is not None and self._ser.is_open

    def connect(self) -> None:
        with self._lock:
            parity = {
                "N": serial.PARITY_NONE,
                "E": serial.PARITY_EVEN,
                "O": serial.PARITY_ODD,
            }.get(self.parity[:1], serial.PARITY_EVEN)
            try:
                self._ser = serial.Serial(
                    port=self.port,
                    baudrate=self.baudrate,
                    bytesize=self.bytesize,
                    parity=parity,
                    stopbits=self.stopbits,
                    timeout=self.timeout,
                )
            except Exception as exc:  # noqa: BLE001
                self._connected = False
                self._ser = None
                raise ConnectionError(f"Failed to open serial port {self.port}: {exc}") from exc
            self._connected = True
            logger.info("A6 RTU connected on %s @ %s 8%s1", self.port, self.baudrate, self.parity)

    def disconnect(self) -> None:
        with self._lock:
            try:
                if self._ser is not None:
                    self._ser.close()
            finally:
                self._ser = None
                self._connected = False
                logger.info("Modbus disconnected")

    def _ensure(self) -> serial.Serial:
        if not self.connected or self._ser is None:
            raise ConnectionError("Modbus bus is not connected")
        return self._ser

    def _txn(self, unit: int, pdu_body: bytes, expect_len: int) -> bytes:
        """Send [unit]+pdu_body+CRC and read expect_len bytes."""
        ser = self._ensure()
        pdu = bytes([unit & 0xFF]) + pdu_body
        frame = pdu + bytes([crc16(pdu) & 0xFF, (crc16(pdu) >> 8) & 0xFF])
        ser.reset_input_buffer()
        ser.write(frame)
        if self.inter_frame_s:
            time.sleep(self.inter_frame_s)
        deadline = time.monotonic() + self.timeout
        buf = bytearray()
        while len(buf) < expect_len and time.monotonic() < deadline:
            chunk = ser.read(expect_len - len(buf))
            if chunk:
                buf.extend(chunk)
        resp = bytes(buf)
        if len(resp) < 3:
            raise ModbusException(f"no/short reply to {pdu.hex(' ')} (got {resp.hex(' ')})")
        if resp[0] != (unit & 0xFF):
            raise ModbusException(f"wrong unit in reply {resp.hex(' ')}")
        if resp[1] & 0x80:
            code = resp[2] if len(resp) > 2 else -1
            raise ModbusException(f"Modbus exception {code} for {pdu.hex(' ')}")
        body, got_crc = resp[:-2], resp[-2] | (resp[-1] << 8)
        if crc16(body) != got_crc:
            raise ModbusException(f"CRC error in reply {resp.hex(' ')}")
        return resp

    def write_u16(self, slave: int, address: int, value: int) -> None:
        val = int(value) & 0xFFFF
        pdu = bytes(
            [
                0x06,
                (address >> 8) & 0xFF,
                address & 0xFF,
                (val >> 8) & 0xFF,
                val & 0xFF,
            ]
        )
        with self._lock:
            self._txn(slave, pdu, 8)

    def write_i16(self, slave: int, address: int, value: int) -> None:
        value = int(value)
        if value < -32768 or value > 32767:
            raise ValueError(f"I16 out of range: {value}")
        self.write_u16(slave, address, value & 0xFFFF)

    def write_u32(self, slave: int, address: int, value: int, *, high_first: bool = False) -> None:
        """FC10 two registers. Default LOW word first (proven A6 path)."""
        u32 = int(value) & 0xFFFFFFFF
        low = u32 & 0xFFFF
        high = (u32 >> 16) & 0xFFFF
        if high_first:
            w0, w1 = high, low
        else:
            w0, w1 = low, high
        pdu = bytes(
            [
                0x10,
                (address >> 8) & 0xFF,
                address & 0xFF,
                0x00,
                0x02,
                0x04,
                (w0 >> 8) & 0xFF,
                w0 & 0xFF,
                (w1 >> 8) & 0xFF,
                w1 & 0xFF,
            ]
        )
        with self._lock:
            self._txn(slave, pdu, 8)

    def write_i32(self, slave: int, address: int, value: int, *, high_first: bool = False) -> None:
        self.write_u32(slave, address, int(value), high_first=high_first)

    def read_holding(self, slave: int, address: int, count: int) -> Sequence[int]:
        pdu = bytes(
            [
                0x03,
                (address >> 8) & 0xFF,
                address & 0xFF,
                (count >> 8) & 0xFF,
                count & 0xFF,
            ]
        )
        with self._lock:
            resp = self._txn(slave, pdu, 3 + 2 * count + 2)
        byte_count = resp[2]
        data = resp[3 : 3 + byte_count]
        return [(data[i] << 8) | data[i + 1] for i in range(0, len(data), 2)]

    def read_u16(self, slave: int, address: int) -> int:
        return int(self.read_holding(slave, address, 1)[0]) & 0xFFFF

    def read_i16(self, slave: int, address: int) -> int:
        raw = self.read_u16(slave, address)
        return raw - 0x10000 if raw >= 0x8000 else raw

    def read_u32(self, slave: int, address: int, *, high_first: bool = False) -> int:
        regs = self.read_holding(slave, address, 2)
        a, b = int(regs[0]) & 0xFFFF, int(regs[1]) & 0xFFFF
        if high_first:
            return (a << 16) | b
        return a | (b << 16)

    def read_i32(self, slave: int, address: int, *, high_first: bool = False) -> int:
        raw = self.read_u32(slave, address, high_first=high_first)
        return raw - 0x100000000 if raw >= 0x80000000 else raw
