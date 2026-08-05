#!/usr/bin/env python3
"""Entry point: Raspberry Pi 400 lathe control GUI (Modbus RTU)."""

from __future__ import annotations

import logging
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from gui.app import run_app


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    run_app(config_path=ROOT / "config.yaml")


if __name__ == "__main__":
    main()
