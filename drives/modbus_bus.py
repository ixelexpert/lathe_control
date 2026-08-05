"""Shared Modbus RTU client for A6-RS servo drives."""

from __future__ import annotations

import logging
import threading
from typing import Sequence

from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusException

logger = logging.getLogger(__name__)


class ModbusBus:
    """Thread-safe Modbus RTU bus for one or more A6-RS drives.

    A6 register addresses map directly from parameter codes:
    C03.0C -> address 0x030C, C12.0A -> 0x120A, etc.
    """

    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        *,
        parity: str = "N",
        stopbits: int = 1,
        bytesize: int = 8,
        timeout: float = 1.0,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self._client = ModbusSerialClient(
            port=port,
            baudrate=baudrate,
            parity=parity,
            stopbits=stopbits,
            bytesize=bytesize,
            timeout=timeout,
        )
        self._lock = threading.RLock()
        self._connected = False

    @property
    def connected(self) -> bool:
        return self._connected

    def connect(self) -> None:
        with self._lock:
            ok = self._client.connect()
            if not ok:
                self._connected = False
                raise ConnectionError(f"Failed to open serial port {self.port}")
            self._connected = True
            logger.info("Modbus connected on %s @ %s", self.port, self.baudrate)

    def disconnect(self) -> None:
        with self._lock:
            try:
                self._client.close()
            finally:
                self._connected = False
                logger.info("Modbus disconnected")

    def _ensure(self) -> None:
        if not self._connected:
            raise ConnectionError("Modbus bus is not connected")

    def write_u16(self, slave: int, address: int, value: int) -> None:
        value = int(value) & 0xFFFF
        with self._lock:
            self._ensure()
            result = self._client.write_register(address, value, device_id=slave)
            self._raise_if_error(result, f"write_u16 slave={slave} addr=0x{address:04X}")

    def write_i16(self, slave: int, address: int, value: int) -> None:
        value = int(value)
        if value < -32768 or value > 32767:
            raise ValueError(f"I16 out of range: {value}")
        if value < 0:
            value = (1 << 16) + value
        self.write_u16(slave, address, value)

    def write_u32(self, slave: int, address: int, value: int, *, high_first: bool = True) -> None:
        """Write 32-bit value as two registers. Default high word first (C0A.06=0)."""
        value = int(value) & 0xFFFFFFFF
        high = (value >> 16) & 0xFFFF
        low = value & 0xFFFF
        regs = [high, low] if high_first else [low, high]
        with self._lock:
            self._ensure()
            result = self._client.write_registers(address, regs, device_id=slave)
            self._raise_if_error(result, f"write_u32 slave={slave} addr=0x{address:04X}")

    def write_i32(self, slave: int, address: int, value: int, *, high_first: bool = True) -> None:
        value = int(value)
        if value < 0:
            value = (1 << 32) + value
        self.write_u32(slave, address, value, high_first=high_first)

    def read_u16(self, slave: int, address: int) -> int:
        with self._lock:
            self._ensure()
            result = self._client.read_holding_registers(address, count=1, device_id=slave)
            self._raise_if_error(result, f"read_u16 slave={slave} addr=0x{address:04X}")
            return int(result.registers[0])

    def read_i16(self, slave: int, address: int) -> int:
        raw = self.read_u16(slave, address)
        return raw - 0x10000 if raw >= 0x8000 else raw

    def read_u32(self, slave: int, address: int, *, high_first: bool = True) -> int:
        with self._lock:
            self._ensure()
            result = self._client.read_holding_registers(address, count=2, device_id=slave)
            self._raise_if_error(result, f"read_u32 slave={slave} addr=0x{address:04X}")
            a, b = result.registers[0], result.registers[1]
            if high_first:
                return ((a & 0xFFFF) << 16) | (b & 0xFFFF)
            return ((b & 0xFFFF) << 16) | (a & 0xFFFF)

    def read_i32(self, slave: int, address: int, *, high_first: bool = True) -> int:
        raw = self.read_u32(slave, address, high_first=high_first)
        return raw - 0x100000000 if raw >= 0x80000000 else raw

    def read_holding(self, slave: int, address: int, count: int) -> Sequence[int]:
        with self._lock:
            self._ensure()
            result = self._client.read_holding_registers(address, count=count, device_id=slave)
            self._raise_if_error(result, f"read_holding slave={slave} addr=0x{address:04X}")
            return list(result.registers)

    @staticmethod
    def _raise_if_error(result, context: str) -> None:
        if result is None:
            raise ModbusException(f"No response: {context}")
        if hasattr(result, "isError") and result.isError():
            raise ModbusException(f"Modbus error ({context}): {result}")
