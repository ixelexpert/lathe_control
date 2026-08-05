#!/usr/bin/env python3
"""Live ballscrew step test using the lathe_control stack (close GUI first)."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from config_io import detect_serial_port, load_config, ballscrew_params_from_config
from drives.modbus_bus import ModbusBus
from drives.ballscrew_axis import BallscrewAxis


def main() -> None:
    cfg = load_config(ROOT / "config.yaml")
    port = detect_serial_port() or cfg.get("serial_port") or "/dev/ttyACM0"
    mb = cfg.get("modbus", {})
    bp = ballscrew_params_from_config(cfg)
    print(f"port={port} parity={mb.get('parity','E')} pitch={bp.pitch_mm} slave={bp.slave_id}")

    bus = ModbusBus(
        port,
        baudrate=bp.baud,
        parity=str(mb.get("parity", "E")),
        timeout=float(mb.get("timeout_s", 0.5)),
    )
    bus.connect()
    axis = BallscrewAxis(bus, bp)
    axis.home_here()
    print(f"home set, pos={axis.status.position_mm:.3f} mm enc={axis.status.encoder_pulses}")

    for dist in (5.0, -5.0, 5.0):
        print(f"--- step {dist:+.1f} mm ---")
        before = axis.status.encoder_pulses
        axis.params.axis_speed_mm_s = 10.0
        axis.move_blocking(dist)
        axis.poll()
        after = axis.status.encoder_pulses
        print(
            f"enc {before} -> {after} (delta {after - before}) "
            f"pos_mm={axis.status.position_mm:.3f}"
        )

    bus.disconnect()
    print("DONE")


if __name__ == "__main__":
    main()
