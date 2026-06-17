# R-Theta 2D Plotter — BLDC-Driven Robotic Arm

A 2-DOF planar robotic arm (r-theta mechanism) that draws on an A4 sheet using BLDC motors controlled by ODrive S1 motor drivers, interfaced to a Raspberry Pi 4 over USB.

---

## Table of Contents

- [Mechanism Design](#mechanism-design)
- [Why R-Theta Over Cartesian](#why-r-theta-over-cartesian)
- [Why BLDC Over Stepper](#why-bldc-over-stepper)
- [Hardware](#hardware)
- [Why These Components](#why-these-components)
- [Wiring & Connections](#wiring--connections)
- [Software Architecture](#software-architecture)
- [Setup & Calibration](#setup--calibration)
- [Known Issues & Fixes](#known-issues--fixes)
- [File Structure](#file-structure)

---

## Mechanism Design

This plotter uses a **2-DOF planar serial manipulator** (RR robot):

- **Joint 1 (Shoulder)** — rotates the upper arm about a fixed base
- **Joint 2 (Elbow)** — rotates the forearm relative to the upper arm

The pen tip traces arbitrary 2D paths by coordinating both joints simultaneously. Inverse kinematics (IK) converts XY Cartesian coordinates into joint angles (θ1, θ2), which are commanded to the motors.

```
Base ──[θ1]── Upper Arm (L1 = 32.5 cm) ──[θ2]── Forearm (L2 = 30.0 cm) ── Pen
```

**Inverse Kinematics equations:**
```
d  = (x² + y² − L1² − L2²) / (2 · L1 · L2)
θ2 = acos(d)
θ1 = atan2(y, x) − atan2(L2 · sin(θ2), L1 + L2 · cos(θ2))
```

The elbow joint has a **3:1 gear reduction**, so the actual motor turns commanded to the ODrive are multiplied by the gear ratio:
```
motor_turns = (joint_angle_deg / 360) × gear_ratio
```

---

## Why R-Theta Over Cartesian

Most DIY plotters use a Cartesian mechanism (H-bot or CoreXY) where two motors drive X and Y axes independently along linear rails. This project uses an R-Theta (rotary-rotary) arm instead. Here is why:

### Mechanical Simplicity

A Cartesian plotter requires two sets of linear rails, timing belts, pulleys, belt tensioners, and a carriage. Every component must be precisely aligned or the motion binds. An R-Theta arm needs only two pivot joints — the entire moving structure is two rigid links. There are no rails to align, no belts to tension, and no carriage to build.

### Lower Build Cost and Weight

Linear rails and bearings are expensive. An R-Theta arm uses only the motor shafts as pivots, with the forearm directly mounted to the elbow motor. This makes the build significantly lighter and cheaper.

### Compact Footprint

A Cartesian plotter must be as large as the paper it draws on — the rails span the full width and height of the workspace. An R-Theta arm sits at a fixed base point and reaches out to the paper. The base footprint is just the size of the shoulder motor mount.

### Naturally Suits Circular Paths

Circular and arc paths are natural for a rotary arm — the IK for a circle produces smooth, continuous joint motion. In Cartesian systems, circles require coordinated XY motion that strains the motion controller.

### Trade-offs (honest comparison)

| Property | R-Theta (this project) | Cartesian (CoreXY / H-bot) |
|---|---|---|
| Mechanical complexity | Low — 2 pivot joints | High — rails, belts, carriage |
| Build cost | Low | Moderate to High |
| Workspace shape | Annular (ring-shaped) | Rectangular |
| Speed uniformity | Varies across workspace | Uniform everywhere |
| Singularities | Yes — at full arm extension | None |
| Backlash sensitivity | Higher (gear/joint play) | Lower (belt drive) |
| A4 paper coverage | ✓ Yes | ✓ Yes |

The main disadvantage of R-Theta is the **non-rectangular workspace** — the arm cannot reach very close to the base (minimum reach = L1 − L2) or very far (maximum reach = L1 + L2). For A4 paper placed at ~30 cm from the shoulder, the workspace is more than sufficient.

---

## Why BLDC Over Stepper

Stepper motors are the default choice for most DIY plotters (3D printers, laser cutters, pen plotters). This project uses BLDC (Brushless DC) motors with FOC (Field Oriented Control) instead. Here is a detailed comparison:

### Torque at Speed

Stepper motors lose torque rapidly as speed increases. At high step rates, a stepper may skip steps entirely — this is called step loss and is catastrophic for a plotter because the controller has no idea the motor missed a step, so all subsequent positions are wrong. BLDC motors maintain rated torque across their entire speed range because FOC continuously adjusts the current vector to always be 90° ahead of the rotor, maximizing torque regardless of speed.

### Smoothness at Low Speed

Steppers move in discrete steps (typically 1.8° per step, or 0.9° with half-stepping). Even with microstepping (1/16, 1/32), the motor detents between steps create a **cogging** effect — the motion is not truly smooth but a series of micro-jerks. This appears as jagged lines in a plotter.

BLDC motors with FOC produce **sinusoidal phase currents** that create a continuously rotating magnetic field. The rotor follows this field smoothly with no discrete steps. At any speed, the motion is smooth — this is critical for plotting quality.

### Closed-Loop Position Control

Steppers are almost always run **open-loop** — the controller sends step pulses and assumes the motor followed. If it didn't (due to load, speed, or resonance), there is no way to know. The position error accumulates silently.

BLDC motors with ODrive run **closed-loop** — the encoder measures actual motor position every control cycle (~8 kHz). If the motor is disturbed or lags, the controller corrects immediately. The position error is always known and bounded.

### Resonance

Steppers have a natural resonance frequency (typically 100–200 Hz) where they lose significant torque and can stall. Running a stepper through its resonance band requires careful speed management or active damping. BLDC motors have no such resonance issue.

### Power Efficiency

Steppers draw full rated current at all times — whether moving or holding position — because the holding torque comes from energizing all coils continuously. BLDC motors with FOC draw current proportional to the torque actually required. At rest with no load, the current drops to near zero. This means less heat, longer component life, and lower power supply requirements.

### Summary Table

| Property | Stepper (open-loop) | BLDC + FOC (this project) |
|---|---|---|
| Position feedback | None (open-loop) | Closed-loop, always known |
| Torque at high speed | Drops sharply | Maintained at rated value |
| Motion smoothness | Stepped / cogging | Continuous sinusoidal |
| Step loss risk | Yes — silent, cumulative | Not possible (closed-loop) |
| Resonance problems | Yes | No |
| Power at idle | Full rated current | Near zero |
| Control complexity | Simple (step/dir) | Higher (FOC, encoder, SDK) |
| Cost | Low | Higher |

For a plotter, **smoothness and closed-loop accuracy** outweigh the higher cost and complexity of BLDC. A stepper plotter drawing a circle will show visible facets at the step boundaries; a BLDC plotter draws a true smooth arc.

---

## Hardware

| Component | Model | Quantity | Role |
|---|---|---|---|
| Controller | Raspberry Pi 4 (4GB) | 1 | Runs Python control scripts |
| Motor Driver | ODrive S1 | 2 | FOC motor control, one per joint |
| Shoulder Motor | Tarot MT4008 BLDC | 1 | Shoulder joint, direct drive |
| Elbow Motor | Tarot MT6012 BLDC | 1 | Elbow joint, 3:1 gear reduction |
| Encoder | MA325S (onboard ODrive S1) | 2 | Absolute magnetic, SPI internal |
| Power Supply | 24V / 12A | 1 | Powers both ODrives |
| Interface | USB-C | 2 cables | RPi → ODrive communication |

---

## Why These Components

### Tarot MT6012 (Elbow Motor)

The MT6012 is a gimbal-class BLDC motor with **12 pole pairs**. More pole pairs means more torque ripple cancellation per revolution, producing smoother output — exactly what a plotting arm needs. The large stator diameter gives high torque density, allowing the elbow to carry the forearm and pen weight at full extension without a heavy gearbox.

> ⚠️ **Critical:** Tarot MT6012 = **12 pole pairs**. Setting `pole_pairs = 7` (a common mistake from other Tarot models) causes severe cogging and stepping behavior. Always verify with the motor datasheet.

### ODrive S1 Motor Driver

The ODrive S1 was chosen because it has a **built-in MA325S absolute magnetic encoder** connected internally via SPI. This eliminates all external encoder wiring — no ribbon cables, no connector failures, no alignment issues. The encoder is always connected and always reads correctly.

It also supports the **`odrivetool` Python SDK**, which allows full motor configuration and real-time position control from a Python script on the RPi with no custom protocol implementation.

### Raspberry Pi 4 (USB, not UART)

The RPi 4 was chosen over ESP32 for two reasons. First, it runs a full Linux environment with Python, making it easy to run IK math, motion profiling, and ODrive control in the same script. Second, and more importantly, it has **multiple USB ports**.

This project controls two ODrives. Using UART GPIO pins on the RPi, you can only control **one ODrive per hardware UART port**, and the RPi 4 has only one fully usable hardware UART (the second requires disabling Bluetooth and enabling overlays in `/boot/config.txt`). Even then, multiplexing two ODrives on one UART is not supported by the `odrivetool` protocol.

**USB completely solves this.** Each ODrive connects via its own USB cable and appears as a separate serial device (`/dev/ttyUSB0`, `/dev/ttyUSB1`). The SDK identifies each by serial number:

```python
odrv_shoulder = odrive.find_any(serial_number="396434783331")
odrv_elbow    = odrive.find_any(serial_number="394C34693331")
```

No pin conflicts, no overlay configuration, no protocol issues.

---

## Wiring & Connections

```
Raspberry Pi 4
├── USB Port 0 ──────── ODrive S1  (Shoulder | SN: 396434783331 | fw v0.6.12)
│                            └── Motor terminals A/B/C → Tarot MT4008
│                            └── MA325S encoder (internal SPI, no external wiring)
│                            └── 24V DC power input
│
└── USB Port 1 ──────── ODrive S1  (Elbow | SN: 394C34693331 | fw v0.6.9)
                             └── Motor terminals A/B/C → Tarot MT6012
                             └── MA325S encoder (internal SPI, no external wiring)
                             └── 24V DC power input

24V / 12A Power Supply
├── → ODrive Shoulder (DC bus)
└── → ODrive Elbow    (DC bus)
```

**Current limits configured per motor:**
- `I_bus_soft_max = 6.0 A` — soft warning threshold
- `I_bus_hard_max = 8.0 A` — hard trip threshold
- `motor.current_soft_max = 6.0 A`
- `motor.current_hard_max = 8.0 A`

With two motors, peak draw is ~16A. The 12A PSU is sufficient because both motors rarely peak simultaneously during slow plotting motion.

---

## Software Architecture

### Three Core Files

**`home_arm.py`** — Run this first before any drawing. Physically place the arm in the L-shape reference pose (shoulder = 0°, elbow = 90°), then run the script. It connects to both ODrives, captures the current encoder position as joint zero, and saves the zero references to `arm_state.json`. Every subsequent script loads this file to know where joint zero is.

**`calibrate_workspace.py`** — Run this once to teach the arm the four corners of your A4 sheet. You manually move the arm to each corner and press Enter. The script reads the encoder positions, computes FK to get XY coordinates, and saves the four corner coordinates to `workspace.json`. This defines the valid drawing area.

**`circle_in_workspace.py`** — The drawing script. Loads `arm_state.json` (joint zeros) and `workspace.json` (bounds), validates that the circle fits within the workspace, moves the pen to the start point, waits for you to lower the pen, then traces the circle using interpolated IK moves.

### Motion Pipeline

```
XY target (metres)
    ↓  inverse_kinematics(x, y, L1, L2)
θ1, θ2 (degrees)
    ↓  motor_turns = (deg / 360) × gear_ratio
ODrive input_pos (turns)
    ↓
FOC current control loop @ ~8 kHz
    ↓
Motor torque → joint rotation → pen motion
```

### Key Parameters

```python
L1            = 0.325   # Shoulder link length (m)
L2            = 0.300   # Elbow link length (m)
GEAR_SHOULDER = 1.0     # Direct drive
GEAR_ELBOW    = 3.0     # 3:1 gear reduction

SERIAL_SHOULDER = "396434783331"   # ODrive S1, fw v0.6.12
SERIAL_ELBOW    = "394C34693331"   # ODrive S1, fw v0.6.9
```

---

## Setup & Calibration

### Step 1 — Install ODrive SDK

```bash
pip install odrive
```

### Step 2 — Connect ODrives

Plug both ODrives into RPi USB ports, then verify:
```bash
ls /dev/ttyUSB*
# Expected: /dev/ttyUSB0  /dev/ttyUSB1
```

### Step 3 — First-Time Motor Calibration (run once per ODrive)

```python
import odrive
from odrive.enums import AxisState

odrv = odrive.find_any(serial_number="YOUR_SERIAL_HERE")

# CRITICAL: set encoder to onboard (0), not RS485 (13)
odrv.axis0.config.load_encoder        = 0
odrv.axis0.config.commutation_encoder = 0

# Motor parameters
odrv.axis0.config.motor.pole_pairs                = 12     # Tarot MT6012
odrv.axis0.config.motor.current_hard_max          = 8.0
odrv.axis0.config.motor.current_soft_max          = 6.0
odrv.axis0.config.motor.current_control_bandwidth = 1000.0

# Run calibration — motor will beep then slowly rotate
odrv.axis0.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE

# Wait until state returns to 1 (IDLE)
import time
while odrv.axis0.current_state != AxisState.IDLE:
    time.sleep(0.5)

odrv.save_configuration()
print("Done")
```

### Step 4 — Workspace Calibration (Why You Must Wait at Each Corner)

Run `calibrate_workspace.py`. It will move the arm to each of the four corners of your A4 sheet and ask you to confirm with Enter.

**You must wait at least 1–2 seconds after the arm stops before pressing Enter.** Here is why this is critical:

When the arm reaches a corner and stops commanding motion, the motors do not stop instantaneously. The ODrive uses a trapezoidal trajectory with a deceleration ramp — the arm is still decelerating and settling. During this settling period, the encoder position (`pos_estimate`) is still changing as the PID controller makes small corrections to reach the exact target. If you press Enter while the arm is still settling, the encoder reading you capture is not the true corner position — it is somewhere between the previous position and the final rested position.

This matters because `workspace.json` stores these corner coordinates as the boundaries of your drawing area. If any corner coordinate is slightly wrong, the IK targets computed from workspace bounds will be offset, and every drawing will be shifted or clipped incorrectly. For a 3 cm circle on A4 paper, even a 5 mm error in workspace calibration can push part of the circle outside the valid joint range, causing the arm to clip the path or hit a soft limit.

**Correct procedure at each corner:**
1. Let the arm move to the corner position
2. Wait for the motion to fully stop (no vibration, no sound from motors)
3. Wait an additional 1–2 seconds for PID settling
4. Only then press Enter to record the position

### Step 5 — Homing Before Every Drawing Session

```bash
python3 home_arm.py
```

Physically place the arm in the L-shape pose before running:
- Shoulder = 0° (upper arm horizontal, pointing forward)
- Elbow = 90° (forearm bent 90° upward)

This pose is the reference zero. The script captures encoder positions at this pose and saves to `arm_state.json`. If you skip homing or place the arm in a slightly different pose, all workspace coordinates will be offset by that error.

### Step 6 — Draw

```bash
python3 circle_in_workspace.py
```

When prompted, lower the pen onto the paper, then press Enter. The arm traces the circle and returns to the home position.

---

## Known Issues & Fixes

| Issue | Root Cause | Fix |
|---|---|---|
| `MISSING_ESTIMATE` on arming | `load_encoder = 13` (RS485, not connected) instead of `0` (onboard) | Set `load_encoder = 0`, `commutation_encoder = 0`, recalibrate |
| Motor steps / cogs at any speed | `current_control_bandwidth` too low after encoder type change | Set `current_control_bandwidth = 1000.0` |
| `CURRENT_LIMIT_VIOLATION` during motion | Default bus current limits are `inf` — PSU trips | Set `I_bus_hard_max = 8.0`, `I_bus_soft_max = 6.0` |
| Circle draws as a straight line | Elbow not moving — encoder misconfigured so `pos_estimate` frozen at 0 | Fix encoder assignment (see above), recalibrate |
| Motor doesn't move during calibration | Motor phase wires loose, or wrong encoder set (no position feedback) | Check A/B/C phase connections; verify encoder config |
| Wrong workspace coordinates | Pressed Enter during arm settling — captured mid-settle encoder value | Wait 1–2s after arm stops before pressing Enter at each corner |
| Can only control one motor via UART | RPi 4 has one usable hardware UART port | Use USB — one cable per ODrive, identified by serial number |
| Motor won't arm after encoder change | `observed_encoder_scale_factor` not computed (calibration never run with new encoder) | Always run `FULL_CALIBRATION_SEQUENCE` after any encoder config change |

---

## File Structure

```
.
├── home_arm.py               # Step 1 — homing, saves arm_state.json
├── calibrate_workspace.py    # Step 2 — corner teach-in, saves workspace.json
├── circle_in_workspace.py    # Step 3 — draws circle on paper
├── arm_state.json            # Joint zeros (auto-generated by home_arm.py)
├── workspace.json            # Calibrated corner coordinates (auto-generated)
├── arm_log.csv               # Motion log from last run
├── arm_log.png               # Joint tracking plot from last run
└── README.md
```

---

## License

MIT
