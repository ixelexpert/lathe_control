#!/usr/bin/env python3
"""Probe A6-RS Modbus with minimalmodbus."""

from __future__ import annotations

import minimalmodbus
import serial
from config_io import detect_serial_port


def probe(port: str, baud: int, slave: int) -> int | None:
    try:
        inst = minimalmodbus.Instrument(port, slave, mode=minimalmodbus.MODE_RTU)
        inst.serial.baudrate = baud
        inst.serial.bytesize = 8
        inst.serial.parity = serial.PARITY_NONE
        inst.serial.stopbits = 1
        inst.serial.timeout = 0.5
        inst.clear_buffers_before_each_transaction = True
        inst.close_port_after_each_call = True
        return int(inst.read_register(0x0000, 0, 3, False))
    except Exception as exc:  # noqa: BLE001
        print(f"  slave={slave} baud={baud} fail: {exc}")
        return None


def main() -> None:
    port = detect_serial_port() or "/dev/ttyACM0"
    print(f"Port: {port}")
    for baud in (115200, 9600, 38400, 19200):
        print(f"=== baud={baud}")
        hits = []
        for slave in range(1, 5):
            val = probe(port, baud, slave)
            if val is not None:
                print(f"  HIT slave={slave} C00.00={val}")
                hits.append(slave)
        if hits:
            print("minimalmodbus OK")
            return
    print("NO DRIVE REPLIES with minimalmodbus")


if __name__ == "__main__":
    main()
