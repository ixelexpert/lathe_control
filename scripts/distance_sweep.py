#!/usr/bin/env python3
"""Test which step distances work on the ballscrew."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from config_io import ballscrew_params_from_config, detect_serial_port, load_config
from drives.ballscrew_axis import BallscrewAxis
from drives.modbus_bus import ModbusBus


def main() -> None:
    cfg = load_config(ROOT / "config.yaml")
    port = detect_serial_port() or "/dev/ttyACM0"
    mb = cfg.get("modbus", {})
    bp = ballscrew_params_from_config(cfg)
    bp.axis_speed_mm_s = 15.0
    bp.soft_min_mm = -500
    bp.soft_max_mm = 500
    print(f"port={port} pitch={bp.pitch_mm} ppm={bp.pulses_per_rev/bp.pitch_mm:.1f}")

    bus = ModbusBus(port, baudrate=bp.baud, parity=str(mb.get("parity", "E")), timeout=1.0)
    bus.connect()
    axis = BallscrewAxis(bus, bp)
    axis.home_here()

    for dist in (5.0, 6.0, 10.0, 15.0, 20.0, 30.0, -10.0, -20.0):
        pulses = axis._distance_pulses(dist)
        before = bus.read_i32(bp.slave_id, 0x4016)
        print(f"\n=== {dist:+.1f} mm ({pulses} pulses) ===")
        try:
            axis.move_blocking(dist)
            after = bus.read_i32(bp.slave_id, 0x4016)
            delta = after - before
            print(f"OK delta={delta} (expect ~{pulses}) err={delta - pulses}")
        except Exception as exc:  # noqa: BLE001
            after = bus.read_i32(bp.slave_id, 0x4016)
            print(f"FAIL: {exc}")
            print(f"enc {before}->{after} delta={after - before}")
            try:
                print(
                    "C11.06",
                    bus.read_i32(bp.slave_id, 0x1106),
                    "C11.08",
                    bus.read_u16(bp.slave_id, 0x1108),
                    "fault",
                    bus.read_u16(bp.slave_id, 0x4101),
                )
            except Exception as e2:  # noqa: BLE001
                print("reg dump failed", e2)

    bus.disconnect()


if __name__ == "__main__":
    main()
