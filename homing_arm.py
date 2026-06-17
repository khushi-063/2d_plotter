#!/usr/bin/env python3
"""
home_arm.py
-----------
Homing sequence based on a MANUAL initial pose.

Assumes:
  Before running, you have physically placed the arm in this pose:
      θ1 (shoulder) = 0°    (upper arm horizontal, pointing forward)
      θ2 (elbow)    = +90°  (forearm bent 90° upward — L-shape)

The script:
  1. Connects to both ODrives
  2. Configures motion control (POSITION + TRAP_TRAJ)
  3. Uses STIFF tuning for both joints during homing
  4. Arms motors and captures CURRENT physical pose as joint zero
     (so wherever you placed the arm becomes "0°, 90°")
  5. Reverts to NORMAL tuning for subsequent motion
  6. Optionally moves to park pose
  7. Leaves both axes in CLOSED_LOOP_CONTROL

This matches the reference pose used by calibrate_workspace.py
so workspace coordinates in workspace.json are valid.
"""

import sys
import time
import math

import odrive
from odrive.enums import AxisState, ControlMode, InputMode


# ════════════════════════════════════════════════════════════
# USER CONFIG
# ════════════════════════════════════════════════════════════
SERIAL_SHOULDER = "396434783331"
SERIAL_ELBOW    = "394C34693331"

GEAR_SHOULDER = 1.0   # direct drive
GEAR_ELBOW    = 3.0   # 3:1 reduction

L1 = 0.30
L2 = 0.3250

# The initial pose you will physically set before running this script.
# These become the joint zero reference.
INITIAL_SHOULDER_DEG = 0.0
INITIAL_ELBOW_DEG    = 0.0

# Soft joint limits (deg) — from your workspace calibration
SHOULDER_MIN_DEG = -75.15
SHOULDER_MAX_DEG = -5.09
ELBOW_MIN_DEG    = 65.26
ELBOW_MAX_DEG    = 121.34

# Optional: move to a park pose after homing.
# Set to None to stay at the initial (0°, 90°) pose.
PARK_SHOULDER_DEG = None   # e.g., 10.0 to move 10° from zero
PARK_ELBOW_DEG    = None   # e.g., 80.0

# Move-to-park timing
PARK_MOVE_TIME_S = 4.0


# ════════════════════════════════════════════════════════════
# TUNING — Stiff during homing, normal afterward
# ════════════════════════════════════════════════════════════
# STIFF homing tuning (high gains for crisp position holding)
SHOULDER_HOMING_TUNING = {
    'pos_gain':            150.0,   # ↑ much stiffer
    'vel_gain':            2.5,     # ↑ stronger damping
    'vel_integrator_gain': 6.0,     # ↑ eliminates drift
    'trap_vel_limit':      0.20,    # slow trajectory
    'trap_accel_limit':    1.0,     # gentle accel
    'trap_decel_limit':    1.0,
    'vel_limit':           0.50,
}

ELBOW_HOMING_TUNING = {
    'pos_gain':            40.0,    # ↑ stiffer than normal (20)
    'vel_gain':            0.30,    # ↑ stronger damping
    'vel_integrator_gain': 0.60,
    'trap_vel_limit':      0.50,
    'trap_accel_limit':    2.0,
    'trap_decel_limit':    2.0,
    'vel_limit':           1.0,
}

# NORMAL tuning (softer for plotting motion)
SHOULDER_NORMAL_TUNING = {
    'pos_gain':            100.0,
    'vel_gain':            1.7,
    'vel_integrator_gain': 4.0,
    'trap_vel_limit':      0.20,
    'trap_accel_limit':    1.0,
    'trap_decel_limit':    1.0,
    'vel_limit':           0.50,
}

ELBOW_NORMAL_TUNING = {
    'pos_gain':            20.0,
    'vel_gain':            0.16,
    'vel_integrator_gain': 0.32,
    'trap_vel_limit':      2.5,
    'trap_accel_limit':    10.0,
    'trap_decel_limit':    10.0,
    'vel_limit':           4.0,
}

# Timeouts
CONNECT_TIMEOUT_S = 15.0
ARM_TIMEOUT_S     = 3.0
MOVE_TIMEOUT_S    = 8.0


# ════════════════════════════════════════════════════════════
# KINEMATICS
# ════════════════════════════════════════════════════════════
def forward_kinematics(th1_rad, th2_rad, l1=L1, l2=L2):
    fwd = l1*math.cos(th1_rad) + l2*math.cos(th1_rad + th2_rad)
    lat = l1*math.sin(th1_rad) + l2*math.sin(th1_rad + th2_rad)
    return lat, fwd  # x=lateral (left=negative), y=forward


# ════════════════════════════════════════════════════════════
# ODRIVE JOINT WRAPPER
# ════════════════════════════════════════════════════════════
class ODriveJoint:
    def __init__(self, serial, name, gear, reference_deg, qmin_deg, qmax_deg):
        self.serial = serial
        self.name = name
        self.gear = gear
        self.reference_deg = reference_deg  # what angle this joint is at when armed
        self.qmin_deg = qmin_deg
        self.qmax_deg = qmax_deg
        self.odrv = None
        self.ax = None
        self.zero = 0.0   # motor turns at joint angle = 0°

    def connect(self, timeout=CONNECT_TIMEOUT_S, retries=3):
        last_err = None
        for attempt in range(1, retries + 1):
            try:
                self.odrv = odrive.find_any(serial_number=self.serial, timeout=timeout)
                self.ax = self.odrv.axis0
                print(f"[{self.name}] connected serial={self.serial}")
                return
            except Exception as e:
                last_err = e
                print(f"[{self.name}] connect attempt {attempt}/{retries} "
                      f"failed: {type(e).__name__}")
                time.sleep(1.0)
        raise RuntimeError(f"[{self.name}] could not connect (last error: {last_err})")

    def configure_motion_mode(self):
        """Set control mode + input mode. Must be idle to call."""
        self.ax.requested_state = AxisState.IDLE
        time.sleep(0.2)
        self.odrv.clear_errors()
        self.ax.controller.config.control_mode = ControlMode.POSITION_CONTROL
        self.ax.controller.config.input_mode   = InputMode.TRAP_TRAJ
        print(f"[{self.name}] control mode = POSITION + TRAP_TRAJ")

    def apply_tuning(self, tuning, label=""):
        """Apply PID gains and trajectory limits."""
        self.ax.controller.config.pos_gain            = tuning['pos_gain']
        self.ax.controller.config.vel_gain            = tuning['vel_gain']
        self.ax.controller.config.vel_integrator_gain = tuning['vel_integrator_gain']
        self.ax.controller.config.vel_limit           = tuning['vel_limit']
        self.ax.trap_traj.config.vel_limit            = tuning['trap_vel_limit']
        self.ax.trap_traj.config.accel_limit          = tuning['trap_accel_limit']
        self.ax.trap_traj.config.decel_limit          = tuning['trap_decel_limit']
        if label:
            print(f"[{self.name}] {label}: pos_gain={tuning['pos_gain']}, "
                  f"trap_vel={tuning['trap_vel_limit']}")

    def capture_zero_at_reference(self):
        """
        Capture the CURRENT physical pose as joint = reference_deg.
        After this, the joint will report `reference_deg` until commanded
        elsewhere, and any future angle commands are relative to this zero.
        """
        ref_motor_turns = (self.reference_deg / 360.0) * self.gear
        current_motor_pos = self.ax.pos_estimate
        # zero = motor position that corresponds to joint angle 0
        self.zero = current_motor_pos - ref_motor_turns

        # Pre-load input_pos so we don't jump when armed
        self.ax.controller.input_pos = current_motor_pos
        print(f"[{self.name}] zero captured (current physical pose = "
              f"{self.reference_deg:+.2f}°)")

    def arm(self):
        """Enter closed loop control. Joint zero must be captured first."""
        self.odrv.clear_errors()
        time.sleep(0.1)
        self.ax.requested_state = AxisState.CLOSED_LOOP_CONTROL

        t0 = time.time()
        while time.time() - t0 < ARM_TIMEOUT_S:
            if self.ax.current_state == AxisState.CLOSED_LOOP_CONTROL:
                print(f"[{self.name}] armed ✓ "
                      f"(state=CLOSED_LOOP, joint={self.joint_deg():+.2f}°)")
                return True
            time.sleep(0.05)

        print(f"[{self.name}] arm FAILED. "
              f"state={self.ax.current_state} "
              f"errors=0x{int(self.ax.active_errors):x} "
              f"disarm=0x{int(self.ax.disarm_reason):x}")
        return False

    def idle(self):
        if self.ax is not None:
            self.ax.requested_state = AxisState.IDLE

    def joint_deg(self):
        if self.ax is None:
            return 0.0
        motor_turns = self.ax.pos_estimate - self.zero
        return (motor_turns / self.gear) * 360.0

    def set_joint_deg(self, deg):
        deg = max(self.qmin_deg, min(self.qmax_deg, deg))
        motor_turns = (deg / 360.0) * self.gear
        self.ax.controller.input_pos = self.zero + motor_turns

    def move_to_deg_blocking(self, deg, timeout=MOVE_TIMEOUT_S, tol_deg=1.5):
        self.set_joint_deg(deg)
        target = max(self.qmin_deg, min(self.qmax_deg, deg))
        t0 = time.time()
        while time.time() - t0 < timeout:
            err = abs(self.joint_deg() - target)
            if err <= tol_deg:
                return True
            if int(self.ax.active_errors) != 0:
                raise RuntimeError(
                    f"[{self.name}] active_errors=0x{int(self.ax.active_errors):x}"
                )
            time.sleep(0.05)
        return False


# ════════════════════════════════════════════════════════════
# HOMING SEQUENCE
# ════════════════════════════════════════════════════════════
def main():
    print('═══════════════════════════════════════════════')
    print('  HOMING SEQUENCE — Manual Initial Pose')
    print('═══════════════════════════════════════════════')
    print()
    print('  Before running, manually position the arm at:')
    print(f'    Shoulder (θ1) = {INITIAL_SHOULDER_DEG:+.1f}°  '
          '(upper arm horizontal, forward)')
    print(f'    Elbow    (θ2) = {INITIAL_ELBOW_DEG:+.1f}°  '
          '(forearm bent 90° upward)')
    print()
    print('  This pose will be captured as joint zero, matching')
    print('  the reference used by calibrate_workspace.py.')
    print()

    # Compute expected tip position for verification
    init_th1 = math.radians(INITIAL_SHOULDER_DEG)
    init_th2 = math.radians(INITIAL_ELBOW_DEG)
    tip_x, tip_y = forward_kinematics(init_th1, init_th2)
    print(f'  Expected tip position: '
          f'({tip_x*100:+.2f}, {tip_y*100:+.2f}) cm')
    print(f'  ({math.hypot(tip_x, tip_y)*100:.2f} cm from shoulder)')
    print()

    # User confirmation
    try:
        input('  Press Enter when arm is in the L-shape pose '
              '(or Ctrl+C to abort) > ')
    except (KeyboardInterrupt, EOFError):
        print('\nAborted.')
        return

    # Create joint wrappers
    shoulder = ODriveJoint(
        SERIAL_SHOULDER, "Shoulder", GEAR_SHOULDER,
        INITIAL_SHOULDER_DEG,
        SHOULDER_MIN_DEG, SHOULDER_MAX_DEG
    )
    elbow = ODriveJoint(
        SERIAL_ELBOW, "Elbow", GEAR_ELBOW,
        INITIAL_ELBOW_DEG,
        ELBOW_MIN_DEG, ELBOW_MAX_DEG
    )

    try:
        # ── Step 1: Connect ──────────────────────────────
        print('\n=== STEP 1: CONNECT ===')
        shoulder.connect()
        elbow.connect()

        # ── Step 2: Configure control mode ───────────────
        print('\n=== STEP 2: CONFIGURE CONTROL MODE ===')
        shoulder.configure_motion_mode()
        elbow.configure_motion_mode()

        # ── Step 3: Apply STIFF homing tuning ────────────
        print('\n=== STEP 3: APPLY STIFF HOMING TUNING ===')
        shoulder.apply_tuning(SHOULDER_HOMING_TUNING, label="homing tuning")
        elbow.apply_tuning(ELBOW_HOMING_TUNING, label="homing tuning")

        # ── Step 4: Capture current physical pose as joint zero ──
        print('\n=== STEP 4: CAPTURE JOINT ZERO ===')
        shoulder.capture_zero_at_reference()
        elbow.capture_zero_at_reference()

        # ── Step 5: Arm motors (will hold at current physical pose) ──
        print('\n=== STEP 5: ARM MOTORS ===')
        if not shoulder.arm():
            raise RuntimeError("Failed to arm shoulder")
        if not elbow.arm():
            raise RuntimeError("Failed to arm elbow")

        print()
        print(f'  ✓ Shoulder reports: {shoulder.joint_deg():+.2f}° '
              f'(expected {INITIAL_SHOULDER_DEG:+.2f}°)')
        print(f'  ✓ Elbow    reports: {elbow.joint_deg():+.2f}° '
              f'(expected {INITIAL_ELBOW_DEG:+.2f}°)')

        # ── Step 6: Optionally move to park pose ─────────
        if PARK_SHOULDER_DEG is not None and PARK_ELBOW_DEG is not None:
            print('\n=== STEP 6: MOVE TO PARK POSE ===')
            print(f'  Target: θ1={PARK_SHOULDER_DEG:+.1f}°, '
                  f'θ2={PARK_ELBOW_DEG:+.1f}°')

            n_steps = max(30, int(PARK_MOVE_TIME_S * 60))
            cur_th1 = math.radians(shoulder.joint_deg())
            cur_th2 = math.radians(elbow.joint_deg())
            tgt_th1 = math.radians(PARK_SHOULDER_DEG)
            tgt_th2 = math.radians(PARK_ELBOW_DEG)

            for i in range(1, n_steps + 1):
                f = 0.5 * (1 - math.cos(math.pi * i / n_steps))
                shoulder.set_joint_deg(math.degrees(cur_th1 + (tgt_th1 - cur_th1) * f))
                elbow.set_joint_deg(math.degrees(cur_th2 + (tgt_th2 - cur_th2) * f))
                time.sleep(PARK_MOVE_TIME_S / n_steps)

            time.sleep(0.5)
            print(f'  ✓ Shoulder: {shoulder.joint_deg():+.2f}°')
            print(f'  ✓ Elbow   : {elbow.joint_deg():+.2f}°')

        # ── Step 7: Revert to NORMAL tuning for subsequent motion ──
        print('\n=== STEP 7: REVERT TO NORMAL TUNING ===')
        shoulder.apply_tuning(SHOULDER_NORMAL_TUNING, label="normal tuning")
        elbow.apply_tuning(ELBOW_NORMAL_TUNING, label="normal tuning")

        # ── Done ─────────────────────────────────────────
        print()
        print('═══════════════════════════════════════════════')
        print('  HOMING COMPLETE ✓')
        print('═══════════════════════════════════════════════')
        final_th1 = shoulder.joint_deg()
        final_th2 = elbow.joint_deg()
        final_x, final_y = forward_kinematics(
            math.radians(final_th1), math.radians(final_th2)
        )
        print(f'  Final joint angles: θ1={final_th1:+.2f}°, θ2={final_th2:+.2f}°')
        print(f'  Final tip position: ({final_x*100:+.2f}, {final_y*100:+.2f}) cm')
        print()
        print('  Both axes are armed in CLOSED_LOOP_CONTROL with normal tuning.')
        print('  Joint zero matches calibrate_workspace.py reference.')
        print('  workspace.json coordinates are now valid for plotting.')
        print()
        print('  Ready to run circle_in_workspace.py or gcode_trace.py.')

    except KeyboardInterrupt:
        print('\n[Ctrl-C] aborted')
        shoulder.idle()
        elbow.idle()

    except Exception as e:
        print(f'\nERROR: {e}')
        shoulder.idle()
        elbow.idle()
        sys.exit(1)


if __name__ == '__main__':
    main()#!/usr/bin/env python3
"""
home_arm.py
-----------
Homing sequence based on a MANUAL initial pose.

Assumes:
  Before running, you have physically placed the arm in this pose:
      θ1 (shoulder) = 0°    (upper arm horizontal, pointing forward)
      θ2 (elbow)    = +90°  (forearm bent 90° upward — L-shape)

The script:
  1. Connects to both ODrives
  2. Configures motion control (POSITION + TRAP_TRAJ)
  3. Uses STIFF tuning for both joints during homing
  4. Arms motors and captures CURRENT physical pose as joint zero
     (so wherever you placed the arm becomes "0°, 90°")
  5. Reverts to NORMAL tuning for subsequent motion
  6. Optionally moves to park pose
  7. Leaves both axes in CLOSED_LOOP_CONTROL

This matches the reference pose used by calibrate_workspace.py
so workspace coordinates in workspace.json are valid.
"""

import sys
import time
import math

import odrive
from odrive.enums import AxisState, ControlMode, InputMode


# ════════════════════════════════════════════════════════════
# USER CONFIG
# ════════════════════════════════════════════════════════════
SERIAL_SHOULDER = "396434783331"
SERIAL_ELBOW    = "394C34693331"

GEAR_SHOULDER = 1.0   # direct drive
GEAR_ELBOW    = 3.0   # 3:1 reduction

L1 = 0.30
L2 = 0.30

# The initial pose you will physically set before running this script.
# These become the joint zero reference.
INITIAL_SHOULDER_DEG = 0.0
INITIAL_ELBOW_DEG    = 90.0

# Soft joint limits (deg) — from your workspace calibration
SHOULDER_MIN_DEG = -67.62
SHOULDER_MAX_DEG = 3.68
ELBOW_MIN_DEG    = 68.58
ELBOW_MAX_DEG    = 121.43

# Optional: move to a park pose after homing.
# Set to None to stay at the initial (0°, 90°) pose.
PARK_SHOULDER_DEG = None   # e.g., 10.0 to move 10° from zero
PARK_ELBOW_DEG    = None   # e.g., 80.0

# Move-to-park timing
PARK_MOVE_TIME_S = 4.0


# ════════════════════════════════════════════════════════════
# TUNING — Stiff during homing, normal afterward
# ════════════════════════════════════════════════════════════
# STIFF homing tuning (high gains for crisp position holding)
SHOULDER_HOMING_TUNING = {
    'pos_gain':            150.0,   # ↑ much stiffer
    'vel_gain':            2.5,     # ↑ stronger damping
    'vel_integrator_gain': 6.0,     # ↑ eliminates drift
    'trap_vel_limit':      0.20,    # slow trajectory
    'trap_accel_limit':    1.0,     # gentle accel
    'trap_decel_limit':    1.0,
    'vel_limit':           0.50,
}

ELBOW_HOMING_TUNING = {
    'pos_gain':            40.0,    # ↑ stiffer than normal (20)
    'vel_gain':            0.30,    # ↑ stronger damping
    'vel_integrator_gain': 0.60,
    'trap_vel_limit':      0.50,
    'trap_accel_limit':    2.0,
    'trap_decel_limit':    2.0,
    'vel_limit':           1.0,
}

# NORMAL tuning (softer for plotting motion)
SHOULDER_NORMAL_TUNING = {
    'pos_gain':            100.0,
    'vel_gain':            1.7,
    'vel_integrator_gain': 4.0,
    'trap_vel_limit':      0.20,
    'trap_accel_limit':    1.0,
    'trap_decel_limit':    1.0,
    'vel_limit':           0.50,
}

ELBOW_NORMAL_TUNING = {
    'pos_gain':            20.0,
    'vel_gain':            0.16,
    'vel_integrator_gain': 0.32,
    'trap_vel_limit':      2.5,
    'trap_accel_limit':    10.0,
    'trap_decel_limit':    10.0,
    'vel_limit':           4.0,
}

# Timeouts
CONNECT_TIMEOUT_S = 15.0
ARM_TIMEOUT_S     = 3.0
MOVE_TIMEOUT_S    = 8.0


# ════════════════════════════════════════════════════════════
# KINEMATICS
# ════════════════════════════════════════════════════════════
def forward_kinematics(th1_rad, th2_rad, l1=L1, l2=L2):
    fwd = l1*math.cos(th1_rad) + l2*math.cos(th1_rad + th2_rad)
    lat = l1*math.sin(th1_rad) + l2*math.sin(th1_rad + th2_rad)
    return lat, fwd  # x=lateral (left=negative), y=forward


# ════════════════════════════════════════════════════════════
# ODRIVE JOINT WRAPPER
# ════════════════════════════════════════════════════════════
class ODriveJoint:
    def __init__(self, serial, name, gear, reference_deg, qmin_deg, qmax_deg):
        self.serial = serial
        self.name = name
        self.gear = gear
        self.reference_deg = reference_deg  # what angle this joint is at when armed
        self.qmin_deg = qmin_deg
        self.qmax_deg = qmax_deg
        self.odrv = None
        self.ax = None
        self.zero = 0.0   # motor turns at joint angle = 0°

    def connect(self, timeout=CONNECT_TIMEOUT_S, retries=3):
        last_err = None
        for attempt in range(1, retries + 1):
            try:
                self.odrv = odrive.find_any(serial_number=self.serial, timeout=timeout)
                self.ax = self.odrv.axis0
                print(f"[{self.name}] connected serial={self.serial}")
                return
            except Exception as e:
                last_err = e
                print(f"[{self.name}] connect attempt {attempt}/{retries} "
                      f"failed: {type(e).__name__}")
                time.sleep(1.0)
        raise RuntimeError(f"[{self.name}] could not connect (last error: {last_err})")

    def configure_motion_mode(self):
        """Set control mode + input mode. Must be idle to call."""
        self.ax.requested_state = AxisState.IDLE
        time.sleep(0.2)
        self.odrv.clear_errors()
        self.ax.controller.config.control_mode = ControlMode.POSITION_CONTROL
        self.ax.controller.config.input_mode   = InputMode.TRAP_TRAJ
        print(f"[{self.name}] control mode = POSITION + TRAP_TRAJ")

    def apply_tuning(self, tuning, label=""):
        """Apply PID gains and trajectory limits."""
        self.ax.controller.config.pos_gain            = tuning['pos_gain']
        self.ax.controller.config.vel_gain            = tuning['vel_gain']
        self.ax.controller.config.vel_integrator_gain = tuning['vel_integrator_gain']
        self.ax.controller.config.vel_limit           = tuning['vel_limit']
        self.ax.trap_traj.config.vel_limit            = tuning['trap_vel_limit']
        self.ax.trap_traj.config.accel_limit          = tuning['trap_accel_limit']
        self.ax.trap_traj.config.decel_limit          = tuning['trap_decel_limit']
        if label:
            print(f"[{self.name}] {label}: pos_gain={tuning['pos_gain']}, "
                  f"trap_vel={tuning['trap_vel_limit']}")

    def capture_zero_at_reference(self):
        """
        Capture the CURRENT physical pose as joint = reference_deg.
        After this, the joint will report `reference_deg` until commanded
        elsewhere, and any future angle commands are relative to this zero.
        """
        ref_motor_turns = (self.reference_deg / 360.0) * self.gear
        current_motor_pos = self.ax.pos_estimate
        # zero = motor position that corresponds to joint angle 0
        self.zero = current_motor_pos - ref_motor_turns

        # Pre-load input_pos so we don't jump when armed
        self.ax.controller.input_pos = current_motor_pos
        print(f"[{self.name}] zero captured (current physical pose = "
              f"{self.reference_deg:+.2f}°)")

    def arm(self):
        """Enter closed loop control. Joint zero must be captured first."""
        self.odrv.clear_errors()
        time.sleep(0.1)
        self.ax.requested_state = AxisState.CLOSED_LOOP_CONTROL

        t0 = time.time()
        while time.time() - t0 < ARM_TIMEOUT_S:
            if self.ax.current_state == AxisState.CLOSED_LOOP_CONTROL:
                print(f"[{self.name}] armed ✓ "
                      f"(state=CLOSED_LOOP, joint={self.joint_deg():+.2f}°)")
                return True
            time.sleep(0.05)

        print(f"[{self.name}] arm FAILED. "
              f"state={self.ax.current_state} "
              f"errors=0x{int(self.ax.active_errors):x} "
              f"disarm=0x{int(self.ax.disarm_reason):x}")
        return False

    def idle(self):
        if self.ax is not None:
            self.ax.requested_state = AxisState.IDLE

    def joint_deg(self):
        if self.ax is None:
            return 0.0
        motor_turns = self.ax.pos_estimate - self.zero
        return (motor_turns / self.gear) * 360.0

    def set_joint_deg(self, deg):
        deg = max(self.qmin_deg, min(self.qmax_deg, deg))
        motor_turns = (deg / 360.0) * self.gear
        self.ax.controller.input_pos = self.zero + motor_turns

    def move_to_deg_blocking(self, deg, timeout=MOVE_TIMEOUT_S, tol_deg=1.5):
        self.set_joint_deg(deg)
        target = max(self.qmin_deg, min(self.qmax_deg, deg))
        t0 = time.time()
        while time.time() - t0 < timeout:
            err = abs(self.joint_deg() - target)
            if err <= tol_deg:
                return True
            if int(self.ax.active_errors) != 0:
                raise RuntimeError(
                    f"[{self.name}] active_errors=0x{int(self.ax.active_errors):x}"
                )
            time.sleep(0.05)
        return False


# ════════════════════════════════════════════════════════════
# HOMING SEQUENCE
# ════════════════════════════════════════════════════════════
def main():
    print('═══════════════════════════════════════════════')
    print('  HOMING SEQUENCE — Manual Initial Pose')
    print('═══════════════════════════════════════════════')
    print()
    print('  Before running, manually position the arm at:')
    print(f'    Shoulder (θ1) = {INITIAL_SHOULDER_DEG:+.1f}°  '
          '(upper arm horizontal, forward)')
    print(f'    Elbow    (θ2) = {INITIAL_ELBOW_DEG:+.1f}°  '
          '(forearm bent 90° upward)')
    print()
    print('  This pose will be captured as joint zero, matching')
    print('  the reference used by calibrate_workspace.py.')
    print()

    # Compute expected tip position for verification
    init_th1 = math.radians(INITIAL_SHOULDER_DEG)
    init_th2 = math.radians(INITIAL_ELBOW_DEG)
    tip_x, tip_y = forward_kinematics(init_th1, init_th2)
    print(f'  Expected tip position: '
          f'({tip_x*100:+.2f}, {tip_y*100:+.2f}) cm')
    print(f'  ({math.hypot(tip_x, tip_y)*100:.2f} cm from shoulder)')
    print()

    # User confirmation
    try:
        input('  Press Enter when arm is in the L-shape pose '
              '(or Ctrl+C to abort) > ')
    except (KeyboardInterrupt, EOFError):
        print('\nAborted.')
        return

    # Create joint wrappers
    shoulder = ODriveJoint(
        SERIAL_SHOULDER, "Shoulder", GEAR_SHOULDER,
        INITIAL_SHOULDER_DEG,
        SHOULDER_MIN_DEG, SHOULDER_MAX_DEG
    )
    elbow = ODriveJoint(
        SERIAL_ELBOW, "Elbow", GEAR_ELBOW,
        INITIAL_ELBOW_DEG,
        ELBOW_MIN_DEG, ELBOW_MAX_DEG
    )

    try:
        # ── Step 1: Connect ──────────────────────────────
        print('\n=== STEP 1: CONNECT ===')
        shoulder.connect()
        elbow.connect()

        # ── Step 2: Configure control mode ───────────────
        print('\n=== STEP 2: CONFIGURE CONTROL MODE ===')
        shoulder.configure_motion_mode()
        elbow.configure_motion_mode()

        # ── Step 3: Apply STIFF homing tuning ────────────
        print('\n=== STEP 3: APPLY STIFF HOMING TUNING ===')
        shoulder.apply_tuning(SHOULDER_HOMING_TUNING, label="homing tuning")
        elbow.apply_tuning(ELBOW_HOMING_TUNING, label="homing tuning")

        # ── Step 4: Capture current physical pose as joint zero ──
        print('\n=== STEP 4: CAPTURE JOINT ZERO ===')
        shoulder.capture_zero_at_reference()
        elbow.capture_zero_at_reference()

        # ── Step 5: Arm motors (will hold at current physical pose) ──
        print('\n=== STEP 5: ARM MOTORS ===')
        if not shoulder.arm():
            raise RuntimeError("Failed to arm shoulder")
        if not elbow.arm():
            raise RuntimeError("Failed to arm elbow")

        print()
        print(f'  ✓ Shoulder reports: {shoulder.joint_deg():+.2f}° '
              f'(expected {INITIAL_SHOULDER_DEG:+.2f}°)')
        print(f'  ✓ Elbow    reports: {elbow.joint_deg():+.2f}° '
              f'(expected {INITIAL_ELBOW_DEG:+.2f}°)')

        # ── Step 6: Optionally move to park pose ─────────
        if PARK_SHOULDER_DEG is not None and PARK_ELBOW_DEG is not None:
            print('\n=== STEP 6: MOVE TO PARK POSE ===')
            print(f'  Target: θ1={PARK_SHOULDER_DEG:+.1f}°, '
                  f'θ2={PARK_ELBOW_DEG:+.1f}°')

            n_steps = max(30, int(PARK_MOVE_TIME_S * 60))
            cur_th1 = math.radians(shoulder.joint_deg())
            cur_th2 = math.radians(elbow.joint_deg())
            tgt_th1 = math.radians(PARK_SHOULDER_DEG)
            tgt_th2 = math.radians(PARK_ELBOW_DEG)

            for i in range(1, n_steps + 1):
                f = 0.5 * (1 - math.cos(math.pi * i / n_steps))
                shoulder.set_joint_deg(math.degrees(cur_th1 + (tgt_th1 - cur_th1) * f))
                elbow.set_joint_deg(math.degrees(cur_th2 + (tgt_th2 - cur_th2) * f))
                time.sleep(PARK_MOVE_TIME_S / n_steps)

            time.sleep(0.5)
            print(f'  ✓ Shoulder: {shoulder.joint_deg():+.2f}°')
            print(f'  ✓ Elbow   : {elbow.joint_deg():+.2f}°')

        # ── Step 7: Revert to NORMAL tuning for subsequent motion ──
        print('\n=== STEP 7: REVERT TO NORMAL TUNING ===')
        shoulder.apply_tuning(SHOULDER_NORMAL_TUNING, label="normal tuning")
        elbow.apply_tuning(ELBOW_NORMAL_TUNING, label="normal tuning")

        # ── Done ─────────────────────────────────────────
        print()
        print('═══════════════════════════════════════════════')
        print('  HOMING COMPLETE ✓')
        print('═══════════════════════════════════════════════')
        final_th1 = shoulder.joint_deg()
        final_th2 = elbow.joint_deg()
        final_x, final_y = forward_kinematics(
            math.radians(final_th1), math.radians(final_th2)
        )
        print(f'  Final joint angles: θ1={final_th1:+.2f}°, θ2={final_th2:+.2f}°')
        print(f'  Final tip position: ({final_x*100:+.2f}, {final_y*100:+.2f}) cm')
        print()
        print('  Both axes are armed in CLOSED_LOOP_CONTROL with normal tuning.')
        print('  Joint zero matches calibrate_workspace.py reference.')
        print('  workspace.json coordinates are now valid for plotting.')
        print()
        print('  Ready to run circle_in_workspace.py or gcode_trace.py.')

    except KeyboardInterrupt:
        print('\n[Ctrl-C] aborted')
        shoulder.idle()
        elbow.idle()

    except Exception as e:
        print(f'\nERROR: {e}')
        shoulder.idle()
        elbow.idle()
        sys.exit(1)


if __name__ == '__main__':
    main()