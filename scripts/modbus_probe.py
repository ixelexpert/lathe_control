#!/usr/bin/env python3
"""Probe A6-RS Modbus connectivity."""

from __future__ import annotations

from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusException

PORT = "/dev/ttyACM3"


def try_read(client: ModbusSerialClient, slave: int, addr: int, count: int = 1):
    try:
        r = client.read_holding_registers(addr, count=count, device_id=slave)
        if r is not None and not r.isError():
            return r.registers
    except ModbusException:
        return None
    return None


def main() -> None:
    for baud in (115200, 9600, 38400, 19200, 57600):
        client = ModbusSerialClient(
            port=PORT,
            baudrate=baud,
            parity="N",
            stopbits=1,
            bytesize=8,
            timeout=0.5,
            retries=0,
        )
        ok = client.connect()
        print(f"=== baud={baud} connect={ok}")
        if not ok:
            continue
        hits = []
        for slave in range(1, 8):
            regs = try_read(client, slave, 0x0000)
            if regs is None:
                print(f"  slave={slave} no reply")
                continue
            hits.append(slave)
            print(f"  HIT slave={slave} C00.00={regs}")
            for addr, count in ((0x0A00, 1), (0x0A01, 1), (0x0411, 1), (0x4001, 2)):
                print(f"    {addr:#06x} -> {try_read(client, slave, addr, count)}")
        client.close()
        if hits:
            print("Drives are answering Modbus.")
            return
    print(
        "NO DRIVE REPLIES.\n"
        "Serial adapter opened, but RS485 Modbus is not reaching an A6-RS drive.\n"
        "Check: drive AC power, CN3 pins 4/5/8 (485+/485-/GND), A/B polarity, "
        "baud C0A.01, station C0A.00."
    )


if __name__ == "__main__":
    main()
