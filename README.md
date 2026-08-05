# Lathe Control — Raspberry Pi 400 + StepperOnline A6-400RS

Python GUI to run a lathe-style cycle: **chuck spins** while the **ballscrew feeds**, both over **Modbus RTU**.

## Hardware this software targets

- Raspberry Pi 400 (desktop + Full HD monitor + keyboard/mouse)
- 2× A6M60-400H2A1-M17 motors with A6-400RS drives
- Ballscrew axis: 0–500 mm, 10 mm pitch, max 200 mm/s, CW+, Home Here in GUI
- Chuck axis: 4:1 gearing, max 300 rpm chuck, CCW+, no home
- USB–RS485 adapter daisy-chained to both drive CN3 ports

**Power:** Drive mains are single-phase ~220 VAC (not the 24 V PSU). The 24 V supply is for later I/O.

## Setup on the Pi

```bash
cd lathe_control
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python main.py
```

Edit `config.yaml` (or use **Save Config** in the GUI):

- `serial_port`: usually `/dev/ttyUSB0` (check `ls /dev/ttyUSB*`)
- Matching `baud` on both axes (default `115200`)
- Slave IDs: ballscrew `1`, chuck `2` (set on each drive panel)

### RS485 wiring (CN3)

| Drive CN3 | USB–RS485 |
|-----------|-----------|
| 485+      | A / TXD+  |
| 485−      | B / TXD−  |
| GND       | GND       |

Set unique Modbus station addresses on each drive. Baud on both drives must match the adapter.

## GUI fields

**Ballscrew:** Axis/Motor speed, Accel, Decel, Distance, Duration (calculated), Baud, Gear Ratio, Home Position, Position (live), Pitch, Start/End Delay.

**Chuck:** Axis/Motor speed, Accel, Decel, Duration, Baud, Gear Ratio, Start/End Delay.

**Start Cycle** runs both axes together after each axis’s start delay. `Chuck_Duration = 0` means the chuck runs until the ballscrew move finishes. **E-STOP** disables both drives.

## Commissioning checklist

1. Power drives from AC mains; connect motors/encoders; wire RS485; set slave IDs.
2. Connect in the GUI at low speeds (defaults are conservative).
3. Test **chuck only**: Apply Chuck Params → set a short `Chuck_Duration` → Start Cycle with ballscrew distance `0` only after verifying soft limits, or temporarily use chuck Apply + manual enable during bring-up.
4. Test **ballscrew only** at low axis speed / short distance; confirm CW+ direction; use **Home Here**.
5. Then run a simultaneous low-speed cycle.
6. Raise speeds toward 200 mm/s and 300 rpm only after directions and soft limits are confirmed.

## Safety

- Software E-Stop and window close disable both drives (`C04.11=0`).
- Soft limits on ballscrew travel (default 0–500 mm).
- Hard caps: 200 mm/s ballscrew, 300 rpm chuck.
- A hardware E-Stop wired to drive DI is strongly recommended for production use; this V1 is software-only.
