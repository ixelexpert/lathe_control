"""Load / save lathe_control config.yaml."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from drives.ballscrew_axis import BallscrewParams
from drives.chuck_axis import ChuckParams

DEFAULT_CONFIG = Path(__file__).resolve().parent / "config.yaml"


def load_config(path: Path | str | None = None) -> dict[str, Any]:
    cfg_path = Path(path) if path else DEFAULT_CONFIG
    with cfg_path.open("r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh) or {}
    return data


def save_config(data: dict[str, Any], path: Path | str | None = None) -> None:
    cfg_path = Path(path) if path else DEFAULT_CONFIG
    with cfg_path.open("w", encoding="utf-8") as fh:
        yaml.safe_dump(data, fh, sort_keys=False)


def ballscrew_params_from_config(cfg: dict[str, Any]) -> BallscrewParams:
    b = cfg.get("ballscrew", {})
    limits = cfg.get("limits", {})
    return BallscrewParams(
        slave_id=int(b.get("slave_id", 1)),
        baud=int(b.get("baud", 115200)),
        gear_ratio=float(b.get("gear_ratio", 1.0)),
        pitch_mm=float(b.get("pitch_mm", 10.0)),
        pulses_per_rev=int(b.get("pulses_per_rev", 10000)),
        axis_speed_mm_s=float(b.get("axis_speed_mm_s", 20.0)),
        acceleration_ms=float(b.get("acceleration_ms", 200)),
        deceleration_ms=float(b.get("deceleration_ms", 200)),
        distance_mm=float(b.get("distance_mm", 50.0)),
        home_position_mm=float(b.get("home_position_mm", 0.0)),
        soft_min_mm=float(b.get("soft_min_mm", 0.0)),
        soft_max_mm=float(b.get("soft_max_mm", 500.0)),
        start_delay_s=float(b.get("start_delay_s", 0.0)),
        end_delay_s=float(b.get("end_delay_s", 0.0)),
        invert_direction=bool(b.get("invert_direction", False)),
        max_axis_mm_s=float(limits.get("max_ballscrew_mm_s", 200.0)),
    )


def chuck_params_from_config(cfg: dict[str, Any]) -> ChuckParams:
    c = cfg.get("chuck", {})
    limits = cfg.get("limits", {})
    return ChuckParams(
        slave_id=int(c.get("slave_id", 2)),
        baud=int(c.get("baud", 115200)),
        gear_ratio=float(c.get("gear_ratio", 4.0)),
        axis_speed_rpm=float(c.get("axis_speed_rpm", 60.0)),
        acceleration_ms=float(c.get("acceleration_ms", 200)),
        deceleration_ms=float(c.get("deceleration_ms", 200)),
        duration_s=float(c.get("duration_s", 0.0)),
        start_delay_s=float(c.get("start_delay_s", 0.0)),
        end_delay_s=float(c.get("end_delay_s", 0.0)),
        invert_direction=bool(c.get("invert_direction", False)),
        max_chuck_rpm=float(limits.get("max_chuck_rpm", 300.0)),
    )


def apply_params_to_config(
    cfg: dict[str, Any],
    ballscrew: BallscrewParams,
    chuck: ChuckParams,
    serial_port: str,
) -> dict[str, Any]:
    cfg = dict(cfg)
    cfg["serial_port"] = serial_port
    cfg["ballscrew"] = {
        "slave_id": ballscrew.slave_id,
        "baud": ballscrew.baud,
        "gear_ratio": ballscrew.gear_ratio,
        "pitch_mm": ballscrew.pitch_mm,
        "pulses_per_rev": ballscrew.pulses_per_rev,
        "axis_speed_mm_s": ballscrew.axis_speed_mm_s,
        "acceleration_ms": ballscrew.acceleration_ms,
        "deceleration_ms": ballscrew.deceleration_ms,
        "distance_mm": ballscrew.distance_mm,
        "home_position_mm": ballscrew.home_position_mm,
        "soft_min_mm": ballscrew.soft_min_mm,
        "soft_max_mm": ballscrew.soft_max_mm,
        "start_delay_s": ballscrew.start_delay_s,
        "end_delay_s": ballscrew.end_delay_s,
        "invert_direction": ballscrew.invert_direction,
    }
    cfg["chuck"] = {
        "slave_id": chuck.slave_id,
        "baud": chuck.baud,
        "gear_ratio": chuck.gear_ratio,
        "axis_speed_rpm": chuck.axis_speed_rpm,
        "acceleration_ms": chuck.acceleration_ms,
        "deceleration_ms": chuck.deceleration_ms,
        "duration_s": chuck.duration_s,
        "start_delay_s": chuck.start_delay_s,
        "end_delay_s": chuck.end_delay_s,
        "invert_direction": chuck.invert_direction,
    }
    cfg.setdefault("limits", {})
    cfg["limits"]["max_ballscrew_mm_s"] = ballscrew.max_axis_mm_s
    cfg["limits"]["max_chuck_rpm"] = chuck.max_chuck_rpm
    return cfg
