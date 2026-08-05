"""CustomTkinter lathe control GUI for Pi 400 + A6-400RS."""

from __future__ import annotations

import logging
import threading
import tkinter as tk
from pathlib import Path
from typing import Any

import customtkinter as ctk

from config_io import (
    apply_params_to_config,
    ballscrew_params_from_config,
    chuck_params_from_config,
    load_config,
    resolve_serial_port,
    save_config,
)
from cycle.engine import CycleEngine, CycleState
from drives.ballscrew_axis import BallscrewAxis
from drives.chuck_axis import ChuckAxis
from drives.modbus_bus import ModbusBus
from drives import units

logger = logging.getLogger(__name__)

ctk.set_appearance_mode("light")
ctk.set_default_color_theme("dark-blue")

LAMP_OFF = "#9e9e9e"
LAMP_USB = "#f9a825"      # amber — USB adapter present, Modbus not proven
LAMP_OK = "#2e7d32"       # green — drive Modbus reply received
LAMP_BAD = "#c62828"      # red — connect/probe failed


class StatusLamp(ctk.CTkFrame):
    """Round status lamp with a short label."""

    def __init__(self, master, label: str = "Modbus"):
        super().__init__(master, fg_color="transparent")
        self._canvas = tk.Canvas(self, width=22, height=22, highlightthickness=0, bg=self._bg())
        self._canvas.pack(side="left")
        self._dot = self._canvas.create_oval(3, 3, 19, 19, fill=LAMP_OFF, outline="#424242")
        self._label_var = tk.StringVar(value=label)
        ctk.CTkLabel(self, textvariable=self._label_var, width=90, anchor="w").pack(
            side="left", padx=(6, 0)
        )

    def _bg(self) -> str:
        try:
            return self.master.cget("fg_color")[1] if isinstance(self.master.cget("fg_color"), (list, tuple)) else "#dbdbdb"
        except Exception:  # noqa: BLE001
            return "#dbdbdb"

    def set_state(self, color: str, text: str | None = None) -> None:
        self._canvas.itemconfigure(self._dot, fill=color)
        if text is not None:
            self._label_var.set(text)


class ParamEntry(ctk.CTkFrame):
    def __init__(self, master, label: str, *, readonly: bool = False, width: int = 120):
        super().__init__(master, fg_color="transparent")
        self.readonly = readonly
        self.label = ctk.CTkLabel(self, text=label, anchor="w", width=200)
        self.label.pack(side="left", padx=(0, 8))
        self.var = tk.StringVar(value="")
        state = "disabled" if readonly else "normal"
        self.entry = ctk.CTkEntry(self, textvariable=self.var, width=width, state=state)
        self.entry.pack(side="left")

    def get_float(self) -> float:
        return float(self.var.get().strip())

    def get_int(self) -> int:
        return int(float(self.var.get().strip()))

    def set(self, value: Any) -> None:
        text = f"{value:.4g}" if isinstance(value, float) else str(value)
        if self.readonly:
            self.entry.configure(state="normal")
            self.var.set(text)
            self.entry.configure(state="disabled")
        else:
            self.var.set(text)


class LatheApp(ctk.CTk):
    def __init__(self, config_path: Path | None = None) -> None:
        super().__init__()
        self.title("Lathe Control — Ballscrew + Chuck (Modbus RTU)")
        self.geometry("1280x800")
        self.minsize(1100, 700)

        self.config_path = config_path or Path(__file__).resolve().parent.parent / "config.yaml"
        self.cfg = load_config(self.config_path)

        self.bus: ModbusBus | None = None
        self.ballscrew: BallscrewAxis | None = None
        self.chuck: ChuckAxis | None = None
        self.cycle: CycleEngine | None = None
        self.modbus_ready = False  # True only after a drive answers Modbus

        self._build_ui()
        self._load_fields_from_config()
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(200, self._poll_tick)
        self.after(400, self._startup_usb_and_modbus_check)

    def _build_ui(self) -> None:
        top = ctk.CTkFrame(self)
        top.pack(fill="x", padx=12, pady=10)

        ctk.CTkLabel(top, text="Serial port").pack(side="left", padx=(8, 4))
        configured_port = str(self.cfg.get("serial_port", ""))
        resolved_port, port_note = resolve_serial_port(configured_port)
        self.cfg["serial_port"] = resolved_port
        self.port_var = tk.StringVar(value=resolved_port)
        ctk.CTkEntry(top, textvariable=self.port_var, width=280).pack(side="left")
        ctk.CTkButton(top, text="Rescan", width=70, command=self._rescan_serial).pack(
            side="left", padx=4
        )

        self.modbus_lamp = StatusLamp(top, label="Modbus: …")
        self.modbus_lamp.pack(side="left", padx=(10, 4))
        if Path(resolved_port).exists() or Path(resolved_port).is_symlink():
            self.modbus_lamp.set_state(LAMP_USB, "Modbus: USB only")
        else:
            self.modbus_lamp.set_state(LAMP_BAD, "Modbus: no USB")

        self.btn_connect = ctk.CTkButton(top, text="Connect", width=100, command=self._connect)
        self.btn_connect.pack(side="left", padx=6)
        self.btn_disconnect = ctk.CTkButton(
            top, text="Disconnect", width=100, command=self._disconnect, state="disabled"
        )
        self.btn_disconnect.pack(side="left", padx=4)

        ctk.CTkButton(top, text="Save Config", width=110, command=self._save_config).pack(
            side="left", padx=8
        )
        ctk.CTkButton(
            top,
            text="E-STOP",
            width=110,
            fg_color="#c62828",
            hover_color="#8e0000",
            command=self._estop,
        ).pack(side="left", padx=10)

        self.status_var = tk.StringVar(value=port_note)
        ctk.CTkLabel(top, textvariable=self.status_var, anchor="w").pack(
            side="left", padx=12, fill="x", expand=True
        )

        self.tabs = ctk.CTkTabview(self)
        self.tabs.pack(fill="both", expand=True, padx=12, pady=(0, 8))
        tab_setup = self.tabs.add("Setup / Cycle")
        tab_test = self.tabs.add("Axis Test")

        self._build_setup_tab(tab_setup)
        self._build_test_tab(tab_test)

        note = (
            "Directions: Ballscrew CW+ / CCW− · Chuck CCW+ / CW− · "
            "Use Axis Test for independent jogging · "
            "Chuck_Duration 0 on Setup = spin until ballscrew move completes."
        )
        ctk.CTkLabel(self, text=note, wraplength=1200, anchor="w").pack(
            fill="x", padx=16, pady=(0, 10)
        )

        self._syncing = False
        self._z_step_busy = False

    def _set_modbus_lamp(self, color: str, text: str) -> None:
        self.modbus_lamp.set_state(color, text)

    def _assign_usb_port(self) -> tuple[str, str]:
        port, note = resolve_serial_port(self.port_var.get().strip())
        self.port_var.set(port)
        self.cfg["serial_port"] = port
        try:
            target = Path(port).resolve()
            note = f"{note}"
            if str(target) != port:
                note = f"{note}"
        except Exception:  # noqa: BLE001
            pass
        return port, note

    def _rescan_serial(self) -> None:
        port, note = self._assign_usb_port()
        exists = Path(port).exists() or Path(port).is_symlink()
        if exists and not self.modbus_ready:
            self._set_modbus_lamp(LAMP_USB, "Modbus: USB only")
        elif not exists:
            self._set_modbus_lamp(LAMP_BAD, "Modbus: no USB")
        try:
            target = Path(port).resolve()
            self.status_var.set(f"{note} → {target}")
        except Exception:  # noqa: BLE001
            self.status_var.set(note)
        # Re-check Modbus on the newly assigned port
        self.after(50, self._startup_usb_and_modbus_check)

    def _startup_usb_and_modbus_check(self) -> None:
        """On startup / rescan: assign USB path, then probe Modbus without blocking UI."""
        if self.bus and self.bus.connected:
            return

        port, note = self._assign_usb_port()
        exists = Path(port).exists() or Path(port).is_symlink()
        if not exists:
            self._set_modbus_lamp(LAMP_BAD, "Modbus: no USB")
            self.status_var.set(f"{note} — adapter not found")
            return

        self._set_modbus_lamp(LAMP_USB, "Modbus: checking…")
        self.status_var.set(f"{note} — probing drives…")

        def worker() -> None:
            detail = ""
            ready = False
            try:
                bp = ballscrew_params_from_config(self.cfg)
                cp = chuck_params_from_config(self.cfg)
                mb = self.cfg.get("modbus", {})
                bus = ModbusBus(
                    port,
                    baudrate=bp.baud,
                    parity=str(mb.get("parity", "E")),
                    stopbits=int(mb.get("stopbits", 1)),
                    bytesize=int(mb.get("bytesize", 8)),
                    timeout=0.4,
                )
                bus.connect()
                for sid in (bp.slave_id, cp.slave_id):
                    try:
                        mode = bus.read_u16(sid, 0x0000)
                        detail += f" slave{sid}=OK({mode})"
                        ready = True
                    except Exception as exc:  # noqa: BLE001
                        detail += f" slave{sid}=no-reply"
                        logger.debug("startup probe slave %s: %s", sid, exc)
                bus.disconnect()
            except Exception as exc:  # noqa: BLE001
                detail = str(exc)
                ready = False

            def finish() -> None:
                if ready:
                    self._set_modbus_lamp(LAMP_OK, "Modbus: connected")
                    self.status_var.set(f"USB OK. Modbus reply:{detail}. Click Connect to use axes.")
                else:
                    self._set_modbus_lamp(LAMP_USB, "Modbus: USB only")
                    self.status_var.set(
                        f"USB assigned ({port}) but no drive reply.{detail}. "
                        "Check RS485 wiring/power, then Connect or Rescan."
                    )

            self.after(0, finish)

        threading.Thread(target=worker, name="ModbusProbe", daemon=True).start()

    def _build_setup_tab(self, parent: ctk.CTkFrame) -> None:
        cycle_bar = ctk.CTkFrame(parent, fg_color="transparent")
        cycle_bar.pack(fill="x", pady=(4, 8))
        ctk.CTkButton(
            cycle_bar, text="Start Cycle", width=120, fg_color="#2e7d32", command=self._start_cycle
        ).pack(side="left", padx=4)
        ctk.CTkButton(cycle_bar, text="Stop Cycle", width=100, command=self._stop_cycle).pack(
            side="left", padx=4
        )

        body = ctk.CTkFrame(parent, fg_color="transparent")
        body.pack(fill="both", expand=True)
        body.grid_columnconfigure(0, weight=1)
        body.grid_columnconfigure(1, weight=1)
        body.grid_rowconfigure(0, weight=1)

        left = ctk.CTkScrollableFrame(body, label_text="Ballscrew Axis")
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        right = ctk.CTkScrollableFrame(body, label_text="Chuck Axis")
        right.grid(row=0, column=1, sticky="nsew", padx=(6, 0))

        self.z_fields: dict[str, ParamEntry] = {}
        z_specs = [
            ("axis_speed", "Ballscrew_Axis_Speed (mm/s)"),
            ("motor_speed", "Ballscrew_Motor_Speed (rpm)"),
            ("accel", "Ballscrew_Acceleration (ms)"),
            ("decel", "Ballscrew_Deceleration (ms)"),
            ("distance", "Ballscrew_Distance (mm)"),
            ("duration", "Ballscrew_Duration (s)", True),
            ("baud", "Ballscrew_Baud"),
            ("gear", "Ballscrew_Gear_Ratio"),
            ("home", "Ballscrew_Home_Position (mm)"),
            ("position", "Ballscrew_Position (mm)", True),
            ("pitch", "Ballscrew_Pitch (mm/rev)"),
            ("start_delay", "Ballscrew_Start_Delay (s)"),
            ("end_delay", "Ballscrew_End_Delay (s)"),
            ("live_axis_speed", "Live Axis Speed (mm/s)", True),
            ("live_motor_speed", "Live Motor Speed (rpm)", True),
        ]
        for spec in z_specs:
            key, label = spec[0], spec[1]
            readonly = len(spec) > 2 and spec[2]
            pe = ParamEntry(left, label, readonly=readonly)
            pe.pack(fill="x", pady=3)
            self.z_fields[key] = pe

        z_btns = ctk.CTkFrame(left, fg_color="transparent")
        z_btns.pack(fill="x", pady=10)
        ctk.CTkButton(z_btns, text="Apply Z Params", command=self._apply_z_from_gui).pack(
            side="left", padx=4
        )
        ctk.CTkButton(z_btns, text="Home Here", command=self._home_here).pack(side="left", padx=4)

        self.c_fields: dict[str, ParamEntry] = {}
        c_specs = [
            ("axis_speed", "Chuck_Axis_Speed (rpm)"),
            ("motor_speed", "Chuck_Motor_Speed (rpm)"),
            ("accel", "Chuck_Acceleration (ms)"),
            ("decel", "Chuck_Deceleration (ms)"),
            ("duration", "Chuck_Duration (s)"),
            ("baud", "Chuck_Baud"),
            ("gear", "Chuck_Gear_Ratio"),
            ("start_delay", "Chuck_Start_Delay (s)"),
            ("end_delay", "Chuck_End_Delay (s)"),
            ("live_axis_speed", "Live Chuck Speed (rpm)", True),
            ("live_motor_speed", "Live Motor Speed (rpm)", True),
        ]
        for spec in c_specs:
            key, label = spec[0], spec[1]
            readonly = len(spec) > 2 and spec[2]
            pe = ParamEntry(right, label, readonly=readonly)
            pe.pack(fill="x", pady=3)
            self.c_fields[key] = pe

        c_btns = ctk.CTkFrame(right, fg_color="transparent")
        c_btns.pack(fill="x", pady=10)
        ctk.CTkButton(c_btns, text="Apply Chuck Params", command=self._apply_c_from_gui).pack(
            side="left", padx=4
        )

        self.z_fields["axis_speed"].var.trace_add("write", lambda *_: self._on_z_axis_speed())
        self.z_fields["motor_speed"].var.trace_add("write", lambda *_: self._on_z_motor_speed())
        self.z_fields["distance"].var.trace_add("write", lambda *_: self._update_z_duration())
        self.z_fields["accel"].var.trace_add("write", lambda *_: self._update_z_duration())
        self.z_fields["decel"].var.trace_add("write", lambda *_: self._update_z_duration())
        self.c_fields["axis_speed"].var.trace_add("write", lambda *_: self._on_c_axis_speed())
        self.c_fields["motor_speed"].var.trace_add("write", lambda *_: self._on_c_motor_speed())

    def _build_test_tab(self, parent: ctk.CTkFrame) -> None:
        parent.grid_columnconfigure(0, weight=1)
        parent.grid_columnconfigure(1, weight=1)
        parent.grid_rowconfigure(0, weight=1)

        z_panel = ctk.CTkFrame(parent)
        z_panel.grid(row=0, column=0, sticky="nsew", padx=(0, 8), pady=8)
        c_panel = ctk.CTkFrame(parent)
        c_panel.grid(row=0, column=1, sticky="nsew", padx=(8, 0), pady=8)

        ctk.CTkLabel(
            z_panel, text="Ballscrew — independent step", font=ctk.CTkFont(size=18, weight="bold")
        ).pack(anchor="w", padx=16, pady=(16, 8))

        self.test_z_speed = ParamEntry(z_panel, "Step speed (mm/s)")
        self.test_z_speed.pack(fill="x", padx=16, pady=6)
        self.test_z_distance = ParamEntry(z_panel, "Step distance (mm)")
        self.test_z_distance.pack(fill="x", padx=16, pady=6)
        self.test_z_position = ParamEntry(z_panel, "Position (mm)", readonly=True)
        self.test_z_position.pack(fill="x", padx=16, pady=6)
        self.test_z_live_speed = ParamEntry(z_panel, "Live speed (mm/s)", readonly=True)
        self.test_z_live_speed.pack(fill="x", padx=16, pady=6)

        z_btns = ctk.CTkFrame(z_panel, fg_color="transparent")
        z_btns.pack(fill="x", padx=16, pady=16)
        ctk.CTkButton(
            z_btns, text="Step − (CCW)", width=140, command=lambda: self._test_step_z(-1)
        ).pack(side="left", padx=4)
        ctk.CTkButton(
            z_btns, text="Step + (CW)", width=140, fg_color="#2e7d32", command=lambda: self._test_step_z(1)
        ).pack(side="left", padx=4)
        ctk.CTkButton(z_btns, text="Stop Z", width=100, command=self._test_stop_z).pack(
            side="left", padx=4
        )
        ctk.CTkButton(z_btns, text="Home Here", width=110, command=self._home_here).pack(
            side="left", padx=4
        )

        ctk.CTkLabel(
            z_panel,
            text="Step + moves CW (positive). Step − moves CCW (negative).\n"
            "Distance is always the absolute step size.",
            justify="left",
        ).pack(anchor="w", padx=16, pady=(0, 16))

        ctk.CTkLabel(
            c_panel, text="Chuck — independent on/off", font=ctk.CTkFont(size=18, weight="bold")
        ).pack(anchor="w", padx=16, pady=(16, 8))

        self.test_c_speed = ParamEntry(c_panel, "Chuck speed (rpm)")
        self.test_c_speed.pack(fill="x", padx=16, pady=6)
        self.test_c_live = ParamEntry(c_panel, "Live chuck speed (rpm)", readonly=True)
        self.test_c_live.pack(fill="x", padx=16, pady=6)
        self.test_c_state = ParamEntry(c_panel, "Chuck state", readonly=True)
        self.test_c_state.pack(fill="x", padx=16, pady=6)
        self.test_c_state.set("Off")

        c_btns = ctk.CTkFrame(c_panel, fg_color="transparent")
        c_btns.pack(fill="x", padx=16, pady=16)
        ctk.CTkButton(
            c_btns, text="Chuck ON", width=140, fg_color="#2e7d32", command=self._test_chuck_on
        ).pack(side="left", padx=4)
        ctk.CTkButton(
            c_btns, text="Chuck OFF", width=140, fg_color="#c62828", command=self._test_chuck_off
        ).pack(side="left", padx=4)

        ctk.CTkLabel(
            c_panel,
            text="Chuck runs independently of ballscrew.\nPositive rpm = CCW.",
            justify="left",
        ).pack(anchor="w", padx=16, pady=(0, 16))

    def _load_fields_from_config(self) -> None:
        bp = ballscrew_params_from_config(self.cfg)
        cp = chuck_params_from_config(self.cfg)
        self._syncing = True
        try:
            self.z_fields["axis_speed"].set(bp.axis_speed_mm_s)
            self.z_fields["motor_speed"].set(
                units.ballscrew_axis_speed_to_motor_rpm(
                    bp.axis_speed_mm_s, pitch_mm=bp.pitch_mm, gear_ratio=bp.gear_ratio
                )
            )
            self.z_fields["accel"].set(bp.acceleration_ms)
            self.z_fields["decel"].set(bp.deceleration_ms)
            self.z_fields["distance"].set(bp.distance_mm)
            self.z_fields["baud"].set(bp.baud)
            self.z_fields["gear"].set(bp.gear_ratio)
            self.z_fields["home"].set(bp.home_position_mm)
            self.z_fields["position"].set(0.0)
            self.z_fields["pitch"].set(bp.pitch_mm)
            self.z_fields["start_delay"].set(bp.start_delay_s)
            self.z_fields["end_delay"].set(bp.end_delay_s)
            self.z_fields["duration"].set(
                units.estimate_move_duration_s(
                    bp.distance_mm, bp.axis_speed_mm_s, bp.acceleration_ms, bp.deceleration_ms
                )
            )

            self.c_fields["axis_speed"].set(cp.axis_speed_rpm)
            self.c_fields["motor_speed"].set(
                units.chuck_axis_to_motor_rpm(cp.axis_speed_rpm, cp.gear_ratio)
            )
            self.c_fields["accel"].set(cp.acceleration_ms)
            self.c_fields["decel"].set(cp.deceleration_ms)
            self.c_fields["duration"].set(cp.duration_s)
            self.c_fields["baud"].set(cp.baud)
            self.c_fields["gear"].set(cp.gear_ratio)
            self.c_fields["start_delay"].set(cp.start_delay_s)
            self.c_fields["end_delay"].set(cp.end_delay_s)

            self.test_z_speed.set(min(20.0, abs(bp.axis_speed_mm_s) or 20.0))
            self.test_z_distance.set(10.0)
            self.test_z_position.set(0.0)
            self.test_z_live_speed.set(0.0)
            self.test_c_speed.set(min(60.0, abs(cp.axis_speed_rpm) or 60.0))
            self.test_c_live.set(0.0)
            self.test_c_state.set("Off")
        finally:
            self._syncing = False

    def _on_z_axis_speed(self) -> None:
        if self._syncing:
            return
        try:
            axis = self.z_fields["axis_speed"].get_float()
            pitch = self.z_fields["pitch"].get_float()
            gear = self.z_fields["gear"].get_float()
            motor = units.ballscrew_axis_speed_to_motor_rpm(
                axis, pitch_mm=pitch, gear_ratio=gear
            )
            self._syncing = True
            self.z_fields["motor_speed"].set(motor)
        except ValueError:
            return
        finally:
            self._syncing = False
        self._update_z_duration()

    def _on_z_motor_speed(self) -> None:
        if self._syncing:
            return
        try:
            motor = self.z_fields["motor_speed"].get_float()
            pitch = self.z_fields["pitch"].get_float()
            gear = self.z_fields["gear"].get_float()
            axis = units.ballscrew_motor_rpm_to_axis_speed(
                motor, pitch_mm=pitch, gear_ratio=gear
            )
            self._syncing = True
            self.z_fields["axis_speed"].set(axis)
        except ValueError:
            return
        finally:
            self._syncing = False
        self._update_z_duration()

    def _update_z_duration(self) -> None:
        try:
            d = units.estimate_move_duration_s(
                self.z_fields["distance"].get_float(),
                self.z_fields["axis_speed"].get_float(),
                self.z_fields["accel"].get_float(),
                self.z_fields["decel"].get_float(),
            )
            self.z_fields["duration"].set(d)
        except ValueError:
            pass

    def _on_c_axis_speed(self) -> None:
        if self._syncing:
            return
        try:
            axis = self.c_fields["axis_speed"].get_float()
            gear = self.c_fields["gear"].get_float()
            self._syncing = True
            self.c_fields["motor_speed"].set(units.chuck_axis_to_motor_rpm(axis, gear))
        except ValueError:
            return
        finally:
            self._syncing = False

    def _on_c_motor_speed(self) -> None:
        if self._syncing:
            return
        try:
            motor = self.c_fields["motor_speed"].get_float()
            gear = self.c_fields["gear"].get_float()
            self._syncing = True
            self.c_fields["axis_speed"].set(units.chuck_motor_to_axis_rpm(motor, gear))
        except ValueError:
            return
        finally:
            self._syncing = False

    def _read_gui_into_params(self) -> tuple:
        bp = ballscrew_params_from_config(self.cfg)
        cp = chuck_params_from_config(self.cfg)

        bp.axis_speed_mm_s = self.z_fields["axis_speed"].get_float()
        bp.acceleration_ms = self.z_fields["accel"].get_float()
        bp.deceleration_ms = self.z_fields["decel"].get_float()
        bp.distance_mm = self.z_fields["distance"].get_float()
        bp.baud = self.z_fields["baud"].get_int()
        bp.gear_ratio = self.z_fields["gear"].get_float()
        bp.home_position_mm = self.z_fields["home"].get_float()
        bp.pitch_mm = self.z_fields["pitch"].get_float()
        bp.start_delay_s = self.z_fields["start_delay"].get_float()
        bp.end_delay_s = self.z_fields["end_delay"].get_float()

        cp.axis_speed_rpm = self.c_fields["axis_speed"].get_float()
        cp.acceleration_ms = self.c_fields["accel"].get_float()
        cp.deceleration_ms = self.c_fields["decel"].get_float()
        cp.duration_s = self.c_fields["duration"].get_float()
        cp.baud = self.c_fields["baud"].get_int()
        cp.gear_ratio = self.c_fields["gear"].get_float()
        cp.start_delay_s = self.c_fields["start_delay"].get_float()
        cp.end_delay_s = self.c_fields["end_delay"].get_float()

        if abs(bp.axis_speed_mm_s) > bp.max_axis_mm_s:
            raise ValueError(f"Ballscrew speed exceeds {bp.max_axis_mm_s} mm/s")
        if abs(cp.axis_speed_rpm) > cp.max_chuck_rpm:
            raise ValueError(f"Chuck speed exceeds {cp.max_chuck_rpm} rpm")
        if bp.baud != cp.baud:
            raise ValueError("Ballscrew_Baud and Chuck_Baud must match (shared RS485 bus)")

        return bp, cp

    def _apply_z_from_gui(self) -> None:
        try:
            bp, cp = self._read_gui_into_params()
            if self.ballscrew:
                self.ballscrew.params = bp
                if self.bus and self.bus.connected:
                    self.ballscrew.apply_motion_params()
            self.status_var.set("Ballscrew params applied")
        except Exception as exc:  # noqa: BLE001
            self.status_var.set(f"Z apply error: {exc}")

    def _apply_c_from_gui(self) -> None:
        try:
            bp, cp = self._read_gui_into_params()
            if self.chuck:
                self.chuck.params = cp
                if self.bus and self.bus.connected:
                    self.chuck.apply_motion_params()
            self.status_var.set("Chuck params applied")
        except Exception as exc:  # noqa: BLE001
            self.status_var.set(f"Chuck apply error: {exc}")

    def _connect(self) -> None:
        try:
            bp, cp = self._read_gui_into_params()
            port = self.port_var.get().strip()
            mb = self.cfg.get("modbus", {})
            self.bus = ModbusBus(
                port,
                baudrate=bp.baud,
                parity=str(mb.get("parity", "N")),
                stopbits=int(mb.get("stopbits", 1)),
                bytesize=int(mb.get("bytesize", 8)),
                timeout=float(mb.get("timeout_s", 1.0)),
            )
            self.bus.connect()
            self.ballscrew = BallscrewAxis(self.bus, bp)
            self.chuck = ChuckAxis(self.bus, cp)
            self.cycle = CycleEngine(self.ballscrew, self.chuck)
            self.cycle.on_state = self._on_cycle_state

            # Prove at least one drive answers before calling this "connected"
            reply_ok = False
            reply_detail = ""
            for sid in (bp.slave_id, cp.slave_id):
                try:
                    mode = self.bus.read_u16(sid, 0x0000)
                    reply_ok = True
                    reply_detail += f" slave{sid} C00.00={mode}"
                except Exception as exc:  # noqa: BLE001
                    reply_detail += f" slave{sid}:no-reply({exc})"

            self.btn_connect.configure(state="disabled")
            self.btn_disconnect.configure(state="normal")

            if not reply_ok:
                self.modbus_ready = False
                self._set_modbus_lamp(LAMP_USB, "Modbus: USB only")
                self.status_var.set(
                    f"Serial open on {port}, but NO Modbus reply from drives.{reply_detail}. "
                    "Motion disabled until RS485 works — check AC power, CN3 485+/485-/GND, "
                    "swap A/B if needed, baud C0A.01, station C0A.00."
                )
                return

            try:
                self.ballscrew.configure()
                self.chuck.configure()
            except Exception as cfg_exc:  # noqa: BLE001
                self.modbus_ready = False
                self._set_modbus_lamp(LAMP_USB, "Modbus: config fail")
                self.status_var.set(f"Drive answered, but configure failed: {cfg_exc}")
                return

            self.modbus_ready = True
            self._set_modbus_lamp(LAMP_OK, "Modbus: connected")
            self.status_var.set(f"Connected on {port}.{reply_detail}")
        except Exception as exc:  # noqa: BLE001
            logger.exception("Connect failed")
            self._set_modbus_lamp(LAMP_BAD, "Modbus: failed")
            self.status_var.set(f"Connect failed: {exc}")
            self._disconnect()

    def _disconnect(self) -> None:
        if self.cycle and self.cycle.busy:
            self.cycle.estop()
        for axis in (self.chuck, self.ballscrew):
            if axis is not None:
                try:
                    axis.disable()
                except Exception:  # noqa: BLE001
                    pass
        if self.bus is not None:
            try:
                self.bus.disconnect()
            except Exception:  # noqa: BLE001
                pass
        self.bus = None
        self.ballscrew = None
        self.chuck = None
        self.cycle = None
        self.modbus_ready = False
        self.btn_connect.configure(state="normal")
        self.btn_disconnect.configure(state="disabled")
        port = self.port_var.get().strip()
        if Path(port).exists() or Path(port).is_symlink():
            self._set_modbus_lamp(LAMP_USB, "Modbus: USB only")
        else:
            self._set_modbus_lamp(LAMP_BAD, "Modbus: no USB")
        self.status_var.set("Disconnected")

    def _save_config(self) -> None:
        try:
            bp, cp = self._read_gui_into_params()
            self.cfg = apply_params_to_config(self.cfg, bp, cp, self.port_var.get().strip())
            save_config(self.cfg, self.config_path)
            self.status_var.set(f"Saved {self.config_path}")
        except Exception as exc:  # noqa: BLE001
            self.status_var.set(f"Save failed: {exc}")

    def _start_cycle(self) -> None:
        if not self.modbus_ready or not self.cycle or not self.bus or not self.bus.connected:
            self.status_var.set("Modbus not ready — fix drive RS485 connection before Start Cycle")
            return
        try:
            bp, cp = self._read_gui_into_params()
            self.ballscrew.params = bp
            self.chuck.params = cp
            self.cycle.start()
        except Exception as exc:  # noqa: BLE001
            self.status_var.set(f"Start failed: {exc}")

    def _stop_cycle(self) -> None:
        if self.cycle:
            self.cycle.stop()
        else:
            self.status_var.set("Nothing running")

    def _estop(self) -> None:
        if self.cycle:
            self.cycle.estop()
        else:
            for axis in (self.chuck, self.ballscrew):
                if axis is not None:
                    try:
                        axis.disable()
                    except Exception:  # noqa: BLE001
                        pass
        self.status_var.set("E-STOP — drives disabled")

    def _home_here(self) -> None:
        if not self.ballscrew or not self.bus or not self.bus.connected:
            self.status_var.set("Connect before Home Here")
            return
        try:
            self.ballscrew.home_here()
            self.z_fields["home"].set(self.ballscrew.params.home_position_mm)
            self.z_fields["position"].set(self.ballscrew.status.position_mm)
            self.test_z_position.set(self.ballscrew.status.position_mm)
            self.status_var.set("Home Here set")
        except Exception as exc:  # noqa: BLE001
            self.status_var.set(f"Home Here failed: {exc}")

    def _require_connected(self) -> bool:
        if not self.bus or not self.bus.connected or not self.ballscrew or not self.chuck:
            self.status_var.set("Connect first")
            return False
        if not self.modbus_ready:
            self.status_var.set(
                "Modbus not ready — drives are not answering. "
                "Check RS485 wiring/power; Step/Chuck will not move until a drive replies."
            )
            return False
        if self.cycle and self.cycle.busy:
            self.status_var.set("Stop the cycle before using Axis Test")
            return False
        return True

    def _test_step_z(self, direction: int) -> None:
        if not self._require_connected():
            return
        if self._z_step_busy:
            self.status_var.set("Ballscrew step already running")
            return
        try:
            speed = abs(self.test_z_speed.get_float())
            distance = abs(self.test_z_distance.get_float())
            if distance <= 0:
                raise ValueError("Step distance must be > 0")
            if speed <= 0:
                raise ValueError("Step speed must be > 0")
            signed = distance if direction >= 0 else -distance
        except Exception as exc:  # noqa: BLE001
            self.status_var.set(f"Step settings error: {exc}")
            return

        self._z_step_busy = True
        self.status_var.set(f"Stepping ballscrew {signed:+.3f} mm @ {speed:.3f} mm/s")

        def worker() -> None:
            try:
                assert self.ballscrew is not None
                # Keep setup-tab accel/pitch/limits; override speed for this step only
                self.ballscrew.params.axis_speed_mm_s = speed
                self.ballscrew.move_blocking(signed)
                self.after(0, lambda: self.status_var.set(f"Ballscrew step done ({signed:+.3f} mm)"))
            except Exception as exc:  # noqa: BLE001
                logger.exception("Ballscrew step failed")
                self.after(0, lambda: self.status_var.set(f"Ballscrew step failed: {exc}"))
            finally:
                self._z_step_busy = False

        threading.Thread(target=worker, name="Z-Step", daemon=True).start()

    def _test_stop_z(self) -> None:
        if not self.ballscrew:
            return
        try:
            self.ballscrew.disable()
            self._z_step_busy = False
            self.status_var.set("Ballscrew stopped")
        except Exception as exc:  # noqa: BLE001
            self.status_var.set(f"Ballscrew stop failed: {exc}")

    def _test_chuck_on(self) -> None:
        if not self._require_connected():
            return
        try:
            rpm = self.test_c_speed.get_float()
            self.chuck.set_axis_speed(rpm)
            self.chuck.start()
            self.test_c_state.set("On")
            self.status_var.set(f"Chuck ON @ {rpm:.2f} rpm")
        except Exception as exc:  # noqa: BLE001
            self.status_var.set(f"Chuck ON failed: {exc}")

    def _test_chuck_off(self) -> None:
        if not self.chuck:
            self.status_var.set("Connect first")
            return
        try:
            self.chuck.stop()
            self.test_c_state.set("Off")
            self.status_var.set("Chuck OFF")
        except Exception as exc:  # noqa: BLE001
            self.status_var.set(f"Chuck OFF failed: {exc}")

    def _on_cycle_state(self, state: CycleState, message: str) -> None:
        self.after(0, lambda: self.status_var.set(f"{state.value}: {message}"))

    def _poll_tick(self) -> None:
        try:
            if self.bus and self.bus.connected and self.ballscrew and self.chuck:
                zs = self.ballscrew.poll()
                cs = self.chuck.poll()
                self.z_fields["position"].set(zs.position_mm)
                self.z_fields["live_axis_speed"].set(zs.axis_speed_mm_s)
                self.z_fields["live_motor_speed"].set(zs.motor_rpm)
                self.c_fields["live_axis_speed"].set(cs.axis_rpm)
                self.c_fields["live_motor_speed"].set(cs.motor_rpm)
                self.test_z_position.set(zs.position_mm)
                self.test_z_live_speed.set(zs.axis_speed_mm_s)
                self.test_c_live.set(cs.axis_rpm)
                if cs.spinning or cs.enabled:
                    self.test_c_state.set("On")
                else:
                    self.test_c_state.set("Off")
        except Exception:  # noqa: BLE001
            logger.debug("poll tick error", exc_info=True)
        self.after(200, self._poll_tick)

    def _on_close(self) -> None:
        try:
            self._estop()
            self._disconnect()
        finally:
            self.destroy()


def run_app(config_path: Path | None = None) -> None:
    app = LatheApp(config_path=config_path)
    app.mainloop()
