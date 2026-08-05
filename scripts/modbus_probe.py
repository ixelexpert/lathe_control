#!/usr/bin/env python3
"""Probe A6-RS Modbus with minimalmodbus — defaults match working ballscrew app (8E1)."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import minimalmodbus
import serial
from config_io import detect_serial_port


def probe(port: str, baud: int, parity: str, slave: int) -> int | None:
    try:
        inst = minimalmodbus.Instrument(port, slave, mode=minimalmodbus.MODE_RTU)
        inst.serial.baudrate = baud
        inst.serial.bytesize = 8
        inst.serial.parity = {
            "E": serial.PARITY_EVEN,
            "O": serial.PARITY_ODD,
            "N": serial.PARITY_NONE,
        }.get(parity.upper(), serial.PARITY_EVEN)
        inst.serial.stopbits = 1
        inst.serial.timeout = 0.5
        inst.clear_buffers_before_each_transaction = True
        inst.close_port_after_each_call = True
        return int(inst.read_register(0x0000, 0, 3, False))
    except Exception as exc:  # noqa: BLE001
        print(f"  slave={slave} {baud} 8{parity}1 fail: {exc}")
        return None


def main() -> None:
    port = detect_serial_port() or "/dev/ttyACM0"
    print(f"Port: {port}")
    # Working /home/pi/ballscrew uses 115200 8E1
    for parity in ("E", "N"):
        for baud in (115200, 9600, 38400):
            print(f"=== {baud} 8{parity}1")
            hits = []
            for slave in range(1, 5):
                val = probe(port, baud, parity, slave)
                if val is not None:
                    print(f"  HIT slave={slave} C00.00={val}")
                    hits.append(slave)
            if hits:
                print(f"OK with {baud} 8{parity}1")
                return
    print("NO DRIVE REPLIES")


if __name__ == "__main__":
    main()
