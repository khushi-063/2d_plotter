#!/usr/bin/env python3
"""
calibrate_workspace.py
----------------------
Interactive workspace calibration for a 2-DOF arm.

You:
  1. Place the arm in a known "reference" pose first.
  2. Manually move the pen tip to each of the 4 corners of your
     writing surface, in this order:
        TL -> TR -> BR -> BL
  3. Press Enter at each corner.

The script:
  - reads joint angles at each corner (motors idle, no motion commanded)
  - converts to tip (x,y) via forward kinematics
  - computes joint angle min/max constraints
  - computes writing surface bounding box, center, width, height
  - suggests safe joint limits with margin
  - saves everything to workspace.json

Usage:
    python3 calibrate_workspace.py
"""

import sys
import time
import math
import json

import odrive
from odrive.enums import AxisState, ControlMode, InputMode


# ============================================================
# USER CONFIG
# ============================================================
SERIAL_SHOULDER = "396434783331"
SERIAL_ELBOW    = "394C34693331"

GEAR_SHOULDER = 1.0        # direct drive
GEAR_ELBOW    = 3.0        # 3:1 reduction

L1 = 0.30                  # upper arm length (m)
L2 = 0.3250                  # forearm length    (m)

# Reference pose the user is asked to set as joint zero
REF_SHOULDER_DEG = 0.0
REF_ELBOW_DEG    = 0.0

# Safety margin added when suggesting joint limits (deg)
JOINT_LIMIT_MARGIN_DEG = 5.0

OUT_JSON = "workspace.json"


# ============================================================
# KINEMATICS
# ============================================================
def forward_kinematics(th1_rad, th2_rad, l1=L1, l2=L2):
    """Joint angles -> tip (x, y) in workspace frame.
    x = lateral  (negative = left,  positive = right)
    y = forward  (positive = away from shoulder)
    Motors rotate clockwise-positive so arm_y_raw already has left = negative.
    """
    fwd = l1*math.cos(th1_rad) + l2*math.cos(th1_rad + th2_rad)
    lat = l1*math.sin(th1_rad) + l2*math.sin(th1_rad + th2_rad)
    return lat, fwd  # x=lateral, y=forward


# ============================================================
# ODRIVE JOINT WRAPPER (minimal — read only)
# ============================================================
class ODriveJoint:
    def __init__(self, serial, gear, name, sign=1.0):
        self.serial = serial
        self.gear = gear
        self.name = name
        self.sign = sign
        self.odrv = None
        self.ax = None
        self.zero = 0.0      # motor turns corresponding to joint reference

    def connect(self, timeout=15, retries=3):
        last_err = None
        for attempt in range(1, retries + 1):
            try:
                self.odrv = odrive.find_any(serial_number=self.serial, timeout=timeout)
                self.ax = self.odrv.axis0
                print(f"[{self.name}] connected (serial={self.serial})")
                return
            except Exception as e:
                last_err = e
                print(f"[{self.name}] connect attempt {attempt}/{retries} failed: {type(e).__name__}")
                time.sleep(1.0)
        raise RuntimeError(f"[{self.name}] could not connect: {last_err}")

    def idle(self):
        if self.ax is not None:
            self.ax.requested_state = AxisState.IDLE

    def clear_errors(self):
        if self.odrv is not None:
            self.odrv.clear_errors()

    def raw_motor_turns(self):
        return self.sign * float(self.ax.pos_estimate)

    def capture_reference_at(self, ref_joint_deg):
        """
        Set 'zero' such that the joint angle at the CURRENT physical pose
        equals ref_joint_deg.
        """
        ref_motor_turns = (ref_joint_deg / 360.0) * self.gear
        self.zero = self.raw_motor_turns() - ref_motor_turns

    def joint_deg(self):
        motor_turns = self.raw_motor_turns() - self.zero
        return (motor_turns / self.gear) * 360.0


# ============================================================
# HELPERS
# ============================================================
def prompt(msg):
    """Show message and wait for Enter. Return True on Enter, False on 'q'/'Q'."""
    try:
        ans = input(msg)
    except (EOFError, KeyboardInterrupt):
        return False
    return ans.strip().lower() != 'q'


def read_pose(shoulder, elbow):
    """Read current joint angles + compute tip (x,y)."""
    th1_deg = shoulder.joint_deg()
    th2_deg = elbow.joint_deg()
    x, y = forward_kinematics(math.radians(th1_deg), math.radians(th2_deg))
    return {
        'th1_deg': th1_deg,
        'th2_deg': th2_deg,
        'x_m':     x,
        'y_m':     y,
    }


def capture_corner(name, shoulder, elbow):
    """Prompt user to position arm at the named corner, then capture pose."""
    while True:
        print()
        print(f"  Move pen tip to the {name} corner of the writing surface.")
        ok = prompt(f"  Press Enter when ready (or 'q' to quit) > ")
        if not ok:
            print("  Aborted by user.")
            sys.exit(0)

        # Small delay to make sure user has let go
        time.sleep(0.1)
        pose = read_pose(shoulder, elbow)

        print(f"  Captured {name}:")
        print(f"    θ1 = {pose['th1_deg']:+7.2f}°")
        print(f"    θ2 = {pose['th2_deg']:+7.2f}°")
        print(f"    tip = ({pose['x_m']*100:+6.2f}, {pose['y_m']*100:+6.2f}) cm")

        confirm = input("  Accept this reading? [Y/n/redo] > ").strip().lower()
        if confirm in ('', 'y', 'yes'):
            return pose
        # Anything else -> retry


def compute_results(corners):
    """Compute bounding box, joint constraints, and suggested limits."""
    th1s = [c['th1_deg'] for c in corners.values()]
    th2s = [c['th2_deg'] for c in corners.values()]
    xs   = [c['x_m']     for c in corners.values()]
    ys   = [c['y_m']     for c in corners.values()]

    results = {
        'corners': corners,

        'joint_limits_observed_deg': {
            'shoulder_min': min(th1s),
            'shoulder_max': max(th1s),
            'elbow_min':    min(th2s),
            'elbow_max':    max(th2s),
        },

        'joint_limits_suggested_deg': {
            'shoulder_min': min(th1s) - JOINT_LIMIT_MARGIN_DEG,
            'shoulder_max': max(th1s) + JOINT_LIMIT_MARGIN_DEG,
            'elbow_min':    min(th2s) - JOINT_LIMIT_MARGIN_DEG,
            'elbow_max':    max(th2s) + JOINT_LIMIT_MARGIN_DEG,
        },

        'surface_bbox_m': {
            'x_min': min(xs),
            'x_max': max(xs),
            'y_min': min(ys),
            'y_max': max(ys),
        },

        'surface_center_m': {
            'x': (min(xs) + max(xs)) / 2.0,
            'y': (min(ys) + max(ys)) / 2.0,
        },

        'surface_size_m': {
            'width':  max(xs) - min(xs),
            'height': max(ys) - min(ys),
        },
    }

    # Quadrilateral area (assumes TL, TR, BR, BL order — shoelace formula)
    if all(k in corners for k in ('TL', 'TR', 'BR', 'BL')):
        pts = [
            (corners['TL']['x_m'], corners['TL']['y_m']),
            (corners['TR']['x_m'], corners['TR']['y_m']),
            (corners['BR']['x_m'], corners['BR']['y_m']),
            (corners['BL']['x_m'], corners['BL']['y_m']),
        ]
        area = 0.0
        for i in range(4):
            x1, y1 = pts[i]
            x2, y2 = pts[(i + 1) % 4]
            area += (x1 * y2) - (x2 * y1)
        results['surface_area_m2'] = abs(area) / 2.0

        # Edge lengths
        def dist(a, b):
            return math.hypot(b[0]-a[0], b[1]-a[1])
        results['edge_lengths_m'] = {
            'top':    dist(pts[0], pts[1]),
            'right':  dist(pts[1], pts[2]),
            'bottom': dist(pts[2], pts[3]),
            'left':   dist(pts[3], pts[0]),
        }

    return results


def print_results(results):
    print()
    print("═══════════════════════════════════════════════")
    print("  CALIBRATION RESULTS")
    print("═══════════════════════════════════════════════")

    print("\nObserved joint angle range:")
    lim_obs = results['joint_limits_observed_deg']
    print(f"  Shoulder: {lim_obs['shoulder_min']:+7.2f}° to {lim_obs['shoulder_max']:+7.2f}°  "
          f"(range {lim_obs['shoulder_max']-lim_obs['shoulder_min']:.2f}°)")
    print(f"  Elbow   : {lim_obs['elbow_min']:+7.2f}° to {lim_obs['elbow_max']:+7.2f}°  "
          f"(range {lim_obs['elbow_max']-lim_obs['elbow_min']:.2f}°)")

    print(f"\nSuggested safe joint limits (+/- {JOINT_LIMIT_MARGIN_DEG:.1f}° margin):")
    lim_sug = results['joint_limits_suggested_deg']
    print(f"  TH1_MIN_DEG = {lim_sug['shoulder_min']:+.2f}")
    print(f"  TH1_MAX_DEG = {lim_sug['shoulder_max']:+.2f}")
    print(f"  TH2_MIN_DEG = {lim_sug['elbow_min']:+.2f}")
    print(f"  TH2_MAX_DEG = {lim_sug['elbow_max']:+.2f}")

    print("\nWriting surface bounding box (in arm coordinates):")
    bb = results['surface_bbox_m']
    print(f"  X: {bb['x_min']*100:+6.2f} cm to {bb['x_max']*100:+6.2f} cm")
    print(f"  Y: {bb['y_min']*100:+6.2f} cm to {bb['y_max']*100:+6.2f} cm")

    print("\nSurface size (axis-aligned):")
    sz = results['surface_size_m']
    print(f"  Width  = {sz['width']*100:.2f} cm")
    print(f"  Height = {sz['height']*100:.2f} cm")

    print(f"\nSurface center: "
          f"({results['surface_center_m']['x']*100:+.2f}, "
          f"{results['surface_center_m']['y']*100:+.2f}) cm")

    if 'edge_lengths_m' in results:
        print("\nMeasured edge lengths (corner-to-corner):")
        e = results['edge_lengths_m']
        print(f"  top    = {e['top']*100:.2f} cm")
        print(f"  right  = {e['right']*100:.2f} cm")
        print(f"  bottom = {e['bottom']*100:.2f} cm")
        print(f"  left   = {e['left']*100:.2f} cm")

    if 'surface_area_m2' in results:
        print(f"\nSurface area = {results['surface_area_m2']*10000:.2f} cm²")

    # Quick sanity check
    print("\nSanity check:")
    expected_aspect = sz['width'] / sz['height'] if sz['height'] > 1e-6 else float('inf')
    print(f"  Width / Height aspect ratio = {expected_aspect:.3f}")
    if 'edge_lengths_m' in results:
        e = results['edge_lengths_m']
        top_bot_diff = abs(e['top'] - e['bottom'])
        side_diff = abs(e['left'] - e['right'])
        print(f"  |top - bottom| = {top_bot_diff*100:.2f} cm "
              f"({'OK' if top_bot_diff < 0.02 else 'WARNING: surface may be tilted'})")
        print(f"  |left - right| = {side_diff*100:.2f} cm "
              f"({'OK' if side_diff < 0.02 else 'WARNING: surface may be tilted'})")

    print("═══════════════════════════════════════════════")


# ============================================================
# MAIN
# ============================================================
def main():
    print("═══ Interactive Workspace Calibration ═══")
    print()
    print("This script will guide you through manually moving the arm")
    print("to the 4 corners of your writing surface.")
    print("Motors will be IDLE the whole time — you move the arm by hand.")
    print()

    # Connect (read-only, no arming)
    shoulder = ODriveJoint(SERIAL_SHOULDER, GEAR_SHOULDER, "Shoulder", sign=-1.0)
    elbow    = ODriveJoint(SERIAL_ELBOW,    GEAR_ELBOW,    "Elbow")

    print("Connecting to ODrives...")
    shoulder.connect()
    elbow.connect()

    # Force IDLE so user can freely move the arm
    print("\nIdling motors so you can move the arm by hand...")
    shoulder.idle()
    elbow.idle()
    shoulder.clear_errors()
    elbow.clear_errors()
    time.sleep(0.5)

    # ── Step 1: capture reference pose ───────────────────────
    print()
    print("─" * 50)
    print(" STEP 1 of 2 — REFERENCE POSE")
    print("─" * 50)
    print(f"  Move the arm so that:")
    print(f"    Shoulder (θ1) = {REF_SHOULDER_DEG:+.1f}°")
    print(f"    Elbow    (θ2) = {REF_ELBOW_DEG:+.1f}°")
    print(f"  (Suggested: upper arm horizontal pointing forward,")
    print(f"   forearm bent 90° upward — L-shape pose)")
    print()
    print("  This pose becomes the reference for all joint angles.")
    ok = prompt("  Press Enter when ready (or 'q' to quit) > ")
    if not ok:
        print("Aborted.")
        return

    shoulder.capture_reference_at(REF_SHOULDER_DEG)
    elbow.capture_reference_at(REF_ELBOW_DEG)

    # Verify by computing initial tip position
    init_pose = read_pose(shoulder, elbow)
    print(f"  ✓ Reference set.")
    print(f"  Reference tip position: ({init_pose['x_m']*100:+.2f}, "
          f"{init_pose['y_m']*100:+.2f}) cm")

    # ── Step 2: capture 4 corners ────────────────────────────
    print()
    print("─" * 50)
    print(" STEP 2 of 2 — CAPTURE 4 CORNERS")
    print("─" * 50)
    print("  Order: TOP-LEFT → TOP-RIGHT → BOTTOM-RIGHT → BOTTOM-LEFT")
    print("  (where 'top' is away from the shoulder, 'left' is the")
    print("   arm's left side)")

    corners = {}
    corners['TL'] = capture_corner("TOP-LEFT     (TL)", shoulder, elbow)
    corners['TR'] = capture_corner("TOP-RIGHT    (TR)", shoulder, elbow)
    corners['BR'] = capture_corner("BOTTOM-RIGHT (BR)", shoulder, elbow)
    corners['BL'] = capture_corner("BOTTOM-LEFT  (BL)", shoulder, elbow)

    # ── Compute & display results ────────────────────────────
    results = compute_results(corners)
    print_results(results)

    # ── Save to JSON ─────────────────────────────────────────
    results['_meta'] = {
        'reference_pose_deg': {
            'shoulder': REF_SHOULDER_DEG,
            'elbow':    REF_ELBOW_DEG,
        },
        'arm_params': {
            'L1_m': L1,
            'L2_m': L2,
            'gear_shoulder': GEAR_SHOULDER,
            'gear_elbow':    GEAR_ELBOW,
        },
        'margin_deg': JOINT_LIMIT_MARGIN_DEG,
        'timestamp':  time.strftime("%Y-%m-%d %H:%M:%S"),
    }

    with open(OUT_JSON, 'w') as f:
        json.dump(results, f, indent=2)
    print(f"\n✓ Saved calibration to {OUT_JSON}")

    # ── Ready-to-paste config for plotter scripts ───────────
    print()
    print("─" * 50)
    print(" PASTE INTO YOUR PLOTTER SCRIPT")
    print("─" * 50)
    lim = results['joint_limits_suggested_deg']
    bb  = results['surface_bbox_m']
    ctr = results['surface_center_m']
    sz  = results['surface_size_m']
    print(f"TH1_MIN_DEG, TH1_MAX_DEG = {lim['shoulder_min']:+.2f}, {lim['shoulder_max']:+.2f}")
    print(f"TH2_MIN_DEG, TH2_MAX_DEG = {lim['elbow_min']:+.2f}, {lim['elbow_max']:+.2f}")
    print(f"WORKSPACE_X_MIN = {bb['x_min']:.4f}    # m")
    print(f"WORKSPACE_X_MAX = {bb['x_max']:.4f}")
    print(f"WORKSPACE_Y_MIN = {bb['y_min']:.4f}")
    print(f"WORKSPACE_Y_MAX = {bb['y_max']:.4f}")
    print(f"WORKSPACE_CENTER_X = {ctr['x']:.4f}")
    print(f"WORKSPACE_CENTER_Y = {ctr['y']:.4f}")
    print(f"WORKSPACE_WIDTH  = {sz['width']:.4f}")
    print(f"WORKSPACE_HEIGHT = {sz['height']:.4f}")
    print()
    print("Done.")


if __name__ == "__main__":
    main()