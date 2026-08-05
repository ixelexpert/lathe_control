"""Shared Modbus RTU bus for A6-RS drives using minimalmodbus."""

from __future__ import annotations

import logging
import threading
from typing import Sequence

import minimalmodbus
import serial

logger = logging.getLogger(__name__)


class ModbusBus:
    """Thread-safe Modbus RTU bus for one or more A6-RS drives.

    Uses minimalmodbus (RTU). A6 register addresses map from parameter codes:
    C03.0C -> 0x030C, C12.0A -> 0x120A, etc.

    Single-register writes use function code 6 (matches StepperOnline examples).
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
        handle_local_echo: bool = False,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.parity = parity
        self.stopbits = stopbits
        self.bytesize = bytesize
        self.timeout = timeout
        self.handle_local_echo = handle_local_echo
        self._lock = threading.RLock()
        self._connected = False
        # One Instrument; slave address is switched per transaction.
        self._instrument: minimalmodbus.Instrument | None = None

    @property
    def connected(self) -> bool:
        return self._connected

    def connect(self) -> None:
        with self._lock:
            try:
                instrument = minimalmodbus.Instrument(
                    self.port,
                    1,
                    mode=minimalmodbus.MODE_RTU,
                    close_port_after_each_call=False,
                    debug=False,
                )
                instrument.serial.baudrate = self.baudrate
                instrument.serial.bytesize = self.bytesize
                instrument.serial.parity = self._parity_const(self.parity)
                instrument.serial.stopbits = self.stopbits
                instrument.serial.timeout = self.timeout
                instrument.clear_buffers_before_each_transaction = True
                instrument.handle_local_echo = self.handle_local_echo
                # Open the port now so failures surface at Connect time
                if not instrument.serial.is_open:
                    instrument.serial.open()
            except Exception as exc:  # noqa: BLE001
                self._connected = False
                self._instrument = None
                raise ConnectionError(f"Failed to open serial port {self.port}: {exc}") from exc

            self._instrument = instrument
            self._connected = True
            logger.info(
                "minimalmodbus RTU connected on %s @ %s", self.port, self.baudrate
            )

    def disconnect(self) -> None:
        with self._lock:
            try:
                if self._instrument is not None and self._instrument.serial.is_open:
                    self._instrument.serial.close()
            finally:
                self._instrument = None
                self._connected = False
                logger.info("Modbus disconnected")

    def _ensure(self) -> minimalmodbus.Instrument:
        if not self._connected or self._instrument is None:
            raise ConnectionError("Modbus bus is not connected")
        return self._instrument

    def _select(self, slave: int) -> minimalmodbus.Instrument:
        instrument = self._ensure()
        instrument.address = int(slave)
        return instrument

    def write_u16(self, slave: int, address: int, value: int) -> None:
        value = int(value) & 0xFFFF
        with self._lock:
            instrument = self._select(slave)
            # FC06 — single register (A6 examples)
            instrument.write_register(
                int(address), value, number_of_decimals=0, functioncode=6, signed=False
            )

    def write_i16(self, slave: int, address: int, value: int) -> None:
        value = int(value)
        if value < -32768 or value > 32767:
            raise ValueError(f"I16 out of range: {value}")
        with self._lock:
            instrument = self._select(slave)
            instrument.write_register(
                int(address), value, number_of_decimals=0, functioncode=6, signed=True
            )

    def write_u32(self, slave: int, address: int, value: int, *, high_first: bool = True) -> None:
        value = int(value) & 0xFFFFFFFF
        high = (value >> 16) & 0xFFFF
        low = value & 0xFFFF
        regs = [high, low] if high_first else [low, high]
        with self._lock:
            instrument = self._select(slave)
            instrument.write_registers(int(address), regs)

    def write_i32(self, slave: int, address: int, value: int, *, high_first: bool = True) -> None:
        value = int(value)
        if value < 0:
            value = (1 << 32) + value
        self.write_u32(slave, address, value, high_first=high_first)

    def read_u16(self, slave: int, address: int) -> int:
        with self._lock:
            instrument = self._select(slave)
            return int(
                instrument.read_register(
                    int(address), number_of_decimals=0, functioncode=3, signed=False
                )
            )

    def read_i16(self, slave: int, address: int) -> int:
        with self._lock:
            instrument = self._select(slave)
            return int(
                instrument.read_register(
                    int(address), number_of_decimals=0, functioncode=3, signed=True
                )
            )

    def read_u32(self, slave: int, address: int, *, high_first: bool = True) -> int:
        with self._lock:
            instrument = self._select(slave)
            regs = instrument.read_registers(int(address), 2, functioncode=3)
            a, b = int(regs[0]), int(regs[1])
            if high_first:
                return ((a & 0xFFFF) << 16) | (b & 0xFFFF)
            return ((b & 0xFFFF) << 16) | (a & 0xFFFF)

    def read_i32(self, slave: int, address: int, *, high_first: bool = True) -> int:
        raw = self.read_u32(slave, address, high_first=high_first)
        return raw - 0x100000000 if raw >= 0x80000000 else raw

    def read_holding(self, slave: int, address: int, count: int) -> Sequence[int]:
        with self._lock:
            instrument = self._select(slave)
            return [int(v) for v in instrument.read_registers(int(address), int(count), functioncode=3)]

    @staticmethod
    def _parity_const(parity: str) -> str:
        p = (parity or "N").upper()[:1]
        if p == "E":
            return serial.PARITY_EVEN
        if p == "O":
            return serial.PARITY_ODD
        return serial.PARITY_NONE
