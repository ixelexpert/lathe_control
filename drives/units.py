"""Unit conversions between axis quantities and motor/drive quantities."""

from __future__ import annotations


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def ballscrew_mm_to_pulses(
    distance_mm: float,
    *,
    pitch_mm: float,
    gear_ratio: float,
    pulses_per_rev: int,
) -> int:
    """Convert linear mm to motor command pulses.

    gear_ratio = motor revolutions / load (ballscrew) revolutions.
    """
    if pitch_mm <= 0:
        raise ValueError("pitch_mm must be > 0")
    load_revs = distance_mm / pitch_mm
    motor_revs = load_revs * gear_ratio
    return int(round(motor_revs * pulses_per_rev))


def ballscrew_pulses_to_mm(
    pulses: int | float,
    *,
    pitch_mm: float,
    gear_ratio: float,
    pulses_per_rev: int,
) -> float:
    if pulses_per_rev <= 0 or gear_ratio == 0:
        raise ValueError("pulses_per_rev and gear_ratio must be non-zero")
    motor_revs = float(pulses) / pulses_per_rev
    load_revs = motor_revs / gear_ratio
    return load_revs * pitch_mm


def ballscrew_axis_speed_to_motor_rpm(
    axis_mm_s: float,
    *,
    pitch_mm: float,
    gear_ratio: float,
) -> float:
    """mm/s at the load -> motor rpm."""
    if pitch_mm <= 0:
        raise ValueError("pitch_mm must be > 0")
    load_rps = axis_mm_s / pitch_mm
    motor_rps = load_rps * gear_ratio
    return motor_rps * 60.0


def ballscrew_motor_rpm_to_axis_speed(
    motor_rpm: float,
    *,
    pitch_mm: float,
    gear_ratio: float,
) -> float:
    if gear_ratio == 0:
        raise ValueError("gear_ratio must be non-zero")
    motor_rps = motor_rpm / 60.0
    load_rps = motor_rps / gear_ratio
    return load_rps * pitch_mm


def chuck_axis_to_motor_rpm(chuck_rpm: float, gear_ratio: float) -> float:
    return chuck_rpm * gear_ratio


def chuck_motor_to_axis_rpm(motor_rpm: float, gear_ratio: float) -> float:
    if gear_ratio == 0:
        raise ValueError("gear_ratio must be non-zero")
    return motor_rpm / gear_ratio


def estimate_move_duration_s(
    distance_mm: float,
    axis_speed_mm_s: float,
    accel_ms: float,
    decel_ms: float,
) -> float:
    """Rough trapezoid estimate for GUI display (not drive-exact)."""
    speed = abs(axis_speed_mm_s)
    dist = abs(distance_mm)
    if speed <= 0:
        return 0.0
    ramp_s = (max(0.0, accel_ms) + max(0.0, decel_ms)) / 1000.0
    # Approximate: half of ramp time is "lost" vs constant-speed estimate
    return dist / speed + ramp_s * 0.5
