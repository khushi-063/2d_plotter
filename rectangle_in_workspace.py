#!/usr/bin/env python3
"""
square_in_workspace.py
"""

import odrive
from odrive.enums import AxisState, ControlMode, InputMode
import math, time, sys, csv, threading

# ════════════════════════════════════════════════════════════
# ARM CONFIG
# ════════════════════════════════════════════════════════════
SERIAL_SHOULDER = "396434783331"
SERIAL_ELBOW    = "394C34693331"

L1 = 0.30   # upper arm — FIXED
L2 = 0.325  # forearm   — FIXED

GEAR_SHOULDER = 1.0
GEAR_ELBOW    = 3.0

START_SHOULDER_DEG = 0.0
START_ELBOW_DEG    = 0.0   # FIXED: must match REF_ELBOW_DEG in calibration

ELBOW_UP = True

# ════════════════════════════════════════════════════════════
# CALIBRATED CONSTRAINTS — pasted fresh from calibrate_workspace.py
# ════════════════════════════════════════════════════════════
TH1_MIN_DEG, TH1_MAX_DEG = -75.15, -5.09
TH2_MIN_DEG, TH2_MAX_DEG = +65.26, +121.34
WORKSPACE_X_MIN = -0.0476    # m
WORKSPACE_X_MAX = 0.2294
WORKSPACE_Y_MIN = 0.2904
WORKSPACE_Y_MAX = 0.4771
WORKSPACE_CENTER_X = 0.0909
WORKSPACE_CENTER_Y = 0.3838
WORKSPACE_WIDTH  = 0.2770
WORKSPACE_HEIGHT = 0.1867

# ════════════════════════════════════════════════════════════
# SQUARE CONFIG
# ════════════════════════════════════════════════════════════
WORKSPACE_MARGIN = 0.01
MAX_SIDE_FIT = min(
    WORKSPACE_WIDTH - (2 * WORKSPACE_MARGIN),
    WORKSPACE_HEIGHT - (2 * WORKSPACE_MARGIN),
)
SIDE_LENGTH = min(0.06, MAX_SIDE_FIT)  # 6cm side by default, scales down if workspace is tight
CENTER_X = WORKSPACE_CENTER_X
CENTER_Y = WORKSPACE_CENTER_Y
N_POINTS = 360 # Points distributed across the 4 sides
N_LOOPS  = 2

# ════════════════════════════════════════════════════════════
# TIMING
# ════════════════════════════════════════════════════════════
LOOP_TIME     = 30.0
MOVE_TO_START = 12.0
RETURN_TIME   = 12.0
SETTLE_TIME   = 1.0

# ════════════════════════════════════════════════════════════
# TUNING
# ════════════════════════════════════════════════════════════
SHOULDER_TUNING = {
    'pos_gain':            100.0,
    'vel_gain':            1.7,
    'vel_integrator_gain': 4.0,
    'trap_vel_limit':      0.20,
    'trap_accel_limit':    1.0,
    'trap_decel_limit':    1.0,
    'vel_limit':           0.50,
}
ELBOW_TUNING = {
    'pos_gain':            100.0,
    'vel_gain':            0.16,
    'vel_integrator_gain': 0.32,
    'trap_vel_limit':      0.6,
    'trap_accel_limit':    1.0,
    'trap_decel_limit':    1.0,
    'vel_limit':           4.0,
}

LOG_RATE_HZ = 50
LOG_CSV     = 'arm_square_log.csv'
LOG_PNG     = 'arm_square_log.png'
TUNING_TXT  = 'tuning_report_square.txt'


# ════════════════════════════════════════════════════════════
# KINEMATICS  — standard convention, matches calibration files
# ════════════════════════════════════════════════════════════
def inverse_kinematics(x, y, l1=L1, l2=L2, elbow_up=True):
    math_x = y
    math_y = x
    d = (math_x*math_x + math_y*math_y - l1*l1 - l2*l2) / (2*l1*l2)
    if abs(d) > 1.0:
        raise ValueError(f"Target ({x:.3f}, {y:.3f}) unreachable")
    th2 = math.acos(d)
    if not elbow_up:
        th2 = -th2
    k1 = l1 + l2*math.cos(th2)
    k2 = l2*math.sin(th2)
    th1 = math.atan2(math_y, math_x) - math.atan2(k2, k1)
    return th1, th2


def forward_kinematics(th1, th2, l1=L1, l2=L2):
    fwd = l1*math.cos(th1) + l2*math.cos(th1 + th2)
    lat = l1*math.sin(th1) + l2*math.sin(th1 + th2)
    return lat, fwd


def check_joint_limits(th1_rad, th2_rad):
    th1_deg = math.degrees(th1_rad)
    th2_deg = math.degrees(th2_rad)
    if not (TH1_MIN_DEG <= th1_deg <= TH1_MAX_DEG):
        raise ValueError(f"shoulder {th1_deg:+.2f}° out of [{TH1_MIN_DEG:+.2f}, {TH1_MAX_DEG:+.2f}]°")
    if not (TH2_MIN_DEG <= th2_deg <= TH2_MAX_DEG):
        raise ValueError(f"elbow {th2_deg:+.2f}° out of [{TH2_MIN_DEG:+.2f}, {TH2_MAX_DEG:+.2f}]°")


def check_workspace_bounds(x, y):
    if not (WORKSPACE_X_MIN <= x <= WORKSPACE_X_MAX):
        raise ValueError(f"x={x*100:+.2f} cm out of workspace")
    if not (WORKSPACE_Y_MIN <= y <= WORKSPACE_Y_MAX):
        raise ValueError(f"y={y*100:+.2f} cm out of workspace")


# ════════════════════════════════════════════════════════════
# ODRIVE — FIXED: normal convention matching calibration files
# ════════════════════════════════════════════════════════════
def configure_odrive(odrv, ax, tuning, name):
    ax.requested_state = AxisState.IDLE
    time.sleep(0.3)
    odrv.clear_errors()
    ax.controller.config.control_mode = ControlMode.POSITION_CONTROL
    ax.controller.config.input_mode   = InputMode.TRAP_TRAJ
    ax.controller.config.pos_gain            = tuning['pos_gain']
    ax.controller.config.vel_gain            = tuning['vel_gain']
    ax.controller.config.vel_integrator_gain = tuning['vel_integrator_gain']
    ax.controller.config.vel_limit           = tuning['vel_limit']
    ax.trap_traj.config.vel_limit            = tuning['trap_vel_limit']
    ax.trap_traj.config.accel_limit          = tuning['trap_accel_limit']
    ax.trap_traj.config.decel_limit          = tuning['trap_decel_limit']
    print(f'[{name}] config applied')


def setup(serial, gear, tuning, name, start_angle_deg=0.0, sign=1.0):
    print(f'[{name}] connecting (serial={serial})...')
    odrv = None
    for attempt in range(1, 4):
        try:
            odrv = odrive.find_any(serial_number=serial, timeout=15)
            break
        except Exception:
            print(f'[{name}] attempt {attempt} failed, retrying...')
            time.sleep(1.0)
    if odrv is None:
        print(f'[{name}] ❌ not found'); sys.exit(1)

    ax = odrv.axis0
    configure_odrive(odrv, ax, tuning, name)
    current_pos = sign * float(ax.pos_estimate)
    ref_motor_turns = (start_angle_deg / 360.0) * gear

    zero = current_pos - ref_motor_turns

    ax.controller.input_pos = sign * current_pos
    time.sleep(0.2)
    ax.requested_state = AxisState.CLOSED_LOOP_CONTROL
    t0 = time.time()
    while time.time() - t0 < 3.0:
        if ax.current_state == AxisState.CLOSED_LOOP_CONTROL:
            print(f'[{name}] ✓ armed (zero @ {zero:.3f})')
            return ax, zero, gear, odrv, sign
        time.sleep(0.05)
    print(f'[{name}] ❌ failed to arm. errors=0x{ax.active_errors:x}')
    sys.exit(1)


def move_rad(ax, zero, gear, angle_rad, qmin_deg, qmax_deg, name, sign):
    angle_deg = math.degrees(angle_rad)
    if angle_deg < qmin_deg or angle_deg > qmax_deg:
        clipped = max(qmin_deg, min(qmax_deg, angle_deg))
        print(f'  ⚠️ [{name}] {angle_deg:+.1f}° → clipped to {clipped:+.1f}°')
        angle_rad = math.radians(clipped)
    motor_turns = angle_rad / (2*math.pi) * gear
    ax.controller.input_pos = sign * (zero + motor_turns)


def get_angle_rad(ax, zero, gear, sign):
    return ((sign * ax.pos_estimate - zero) / gear) * 2*math.pi


# ════════════════════════════════════════════════════════════
# LOGGER
# ════════════════════════════════════════════════════════════
class JointLogger(threading.Thread):
    def __init__(self, joints, rate_hz=LOG_RATE_HZ):
        super().__init__(daemon=True)
        self.ax_s, self.zero_s, self.gear_s, self.sign_s, \
            self.ax_e, self.zero_e, self.gear_e, self.sign_e = joints
        self.period = 1.0 / rate_hz
        self.running = False
        self.samples = []
        self.t_start = None
        self.cmd_th1 = 0.0
        self.cmd_th2 = 0.0
        self.phase   = 'idle'

    def set_cmd(self, th1_rad, th2_rad, phase):
        self.cmd_th1 = math.degrees(th1_rad)
        self.cmd_th2 = math.degrees(th2_rad)
        self.phase = phase

    def run(self):
        self.t_start = time.time()
        self.running = True
        next_t = self.t_start
        while self.running:
            th1 = math.degrees(get_angle_rad(self.ax_s, self.zero_s, self.gear_s, self.sign_s))
            th2 = math.degrees(get_angle_rad(self.ax_e, self.zero_e, self.gear_e, self.sign_e))
            self.samples.append((time.time() - self.t_start,
                                 th1, th2, self.cmd_th1, self.cmd_th2, self.phase))
            next_t += self.period
            sleep = next_t - time.time()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.time()

    def stop(self):
        self.running = False
        self.join(timeout=1.0)

    def save_csv(self, path):
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t_s','th1_actual_deg','th2_actual_deg',
                        'th1_cmd_deg','th2_cmd_deg','phase'])
            for row in self.samples:
                w.writerow(row)
        print(f'✓ Logged {len(self.samples)} samples → {path}')

    def save_plot(self, path, cx, cy, side_length):
        try:
            import matplotlib; matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            import numpy as np
        except ImportError:
            return
        if not self.samples: return
        t  = [s[0] for s in self.samples]
        a1 = [s[1] for s in self.samples]
        a2 = [s[2] for s in self.samples]
        c1 = [s[3] for s in self.samples]
        c2 = [s[4] for s in self.samples]
        ph = [s[5] for s in self.samples]
        ee_x = [forward_kinematics(math.radians(a1[i]), math.radians(a2[i]))[0]*100 for i in range(len(a1))]
        ee_y = [forward_kinematics(math.radians(a1[i]), math.radians(a2[i]))[1]*100 for i in range(len(a1))]
        phase_colors = {'idle':'gray','move_to_start':'orange','square':'tab:blue','return':'tab:green'}
        fig, ax = plt.subplots(2, 2, figsize=(14, 10))
        ax[0,0].plot(t, c1, 'k--', lw=1, label='cmd', alpha=0.6)
        ax[0,0].plot(t, a1, 'b-',  lw=1, label='actual')
        ax[0,0].axhline(TH1_MIN_DEG, color='r', linestyle=':', alpha=0.4)
        ax[0,0].axhline(TH1_MAX_DEG, color='r', linestyle=':', alpha=0.4)
        ax[0,0].set_title('Shoulder θ1'); ax[0,0].grid(alpha=0.3); ax[0,0].legend()
        ax[0,1].plot(t, c2, 'k--', lw=1, label='cmd', alpha=0.6)
        ax[0,1].plot(t, a2, 'g-',  lw=1, label='actual')
        ax[0,1].axhline(TH2_MIN_DEG, color='r', linestyle=':', alpha=0.4)
        ax[0,1].axhline(TH2_MAX_DEG, color='r', linestyle=':', alpha=0.4)
        ax[0,1].set_title('Elbow θ2'); ax[0,1].grid(alpha=0.3); ax[0,1].legend()
        e1 = [a1[i]-c1[i] for i in range(len(t))]
        e2 = [a2[i]-c2[i] for i in range(len(t))]
        ax[1,0].plot(t, e1, 'b-', lw=1, label='θ1 err')
        ax[1,0].plot(t, e2, 'g-', lw=1, label='θ2 err')
        ax[1,0].set_title('Tracking error'); ax[1,0].grid(alpha=0.3); ax[1,0].legend()
        for pn, color in phase_colors.items():
            xs = [ee_x[i] for i in range(len(t)) if ph[i]==pn]
            ys = [ee_y[i] for i in range(len(t)) if ph[i]==pn]
            if xs: ax[1,1].plot(xs, ys, '.', color=color, markersize=2, label=pn)
        ax[1,1].plot([WORKSPACE_X_MIN*100,WORKSPACE_X_MAX*100,WORKSPACE_X_MAX*100,
                      WORKSPACE_X_MIN*100,WORKSPACE_X_MIN*100],
                     [WORKSPACE_Y_MIN*100,WORKSPACE_Y_MIN*100,WORKSPACE_Y_MAX*100,
                      WORKSPACE_Y_MAX*100,WORKSPACE_Y_MIN*100], 'k--', alpha=0.5)
        
        # Plot ideal square boundary
        half = side_length / 2.0
        sq_x = [cx+half, cx-half, cx-half, cx+half, cx+half]
        sq_y = [cy+half, cy+half, cy-half, cy-half, cy+half]
        ax[1,1].plot(sq_x, sq_y, 'k:', alpha=0.5)
        
        ax[1,1].plot(cx, cy, 'k+', markersize=12, markeredgewidth=2)
        ax[1,1].plot(0, 0, 'rs', markersize=10, label='shoulder')
        ax[1,1].set_aspect('equal'); ax[1,1].set_title('End-effector trace')
        ax[1,1].grid(alpha=0.3); ax[1,1].legend(fontsize=8)
        plt.tight_layout(); plt.savefig(path, dpi=130); plt.close()
        print(f'✓ Plot saved → {path}')


# ════════════════════════════════════════════════════════════
# MOTION
# ════════════════════════════════════════════════════════════
def interpolated_move(joints, logger, target_th1, target_th2,
                      duration, phase_name, n_steps=None):
    ax_s, zero_s, gear_s, sign_s, ax_e, zero_e, gear_e, sign_e = joints
    cur_th1 = get_angle_rad(ax_s, zero_s, gear_s, sign_s)
    cur_th2 = get_angle_rad(ax_e, zero_e, gear_e, sign_e)
    cur_th1 = max(math.radians(TH1_MIN_DEG), min(math.radians(TH1_MAX_DEG), cur_th1))
    cur_th2 = max(math.radians(TH2_MIN_DEG), min(math.radians(TH2_MAX_DEG), cur_th2))
    if n_steps is None:
        n_steps = max(30, int(duration * 60))
    for i in range(1, n_steps + 1):
        f = 0.5 * (1 - math.cos(math.pi * i / n_steps))
        th1 = cur_th1 + (target_th1 - cur_th1) * f
        th2 = cur_th2 + (target_th2 - cur_th2) * f
        logger.set_cmd(th1, th2, phase_name)
        move_rad(ax_s, zero_s, gear_s, th1, TH1_MIN_DEG, TH1_MAX_DEG, 'Shoulder', sign_s)
        move_rad(ax_e, zero_e, gear_e, th2, TH2_MIN_DEG, TH2_MAX_DEG, 'Elbow', sign_e)
        time.sleep(duration / n_steps)


def trace_square(joints, logger, cx, cy, side_length):
    ax_s, zero_s, gear_s, sign_s, ax_e, zero_e, gear_e, sign_e = joints
    
    half = side_length / 2.0
    corners = [
        (cx + half, cy + half), # Top-Right (Start)
        (cx - half, cy + half), # Top-Left
        (cx - half, cy - half), # Bottom-Left
        (cx + half, cy - half), # Bottom-Right
        (cx + half, cy + half)  # Top-Right (End)
    ]
    
    pts_per_side = max(10, N_POINTS // 4)
    total_points = pts_per_side * 4
    dt = LOOP_TIME / total_points
    
    for loop in range(N_LOOPS):
        pt_idx = 0
        for seg in range(4):
            x0, y0 = corners[seg]
            x1, y1 = corners[seg+1]
            for i in range(pts_per_side):
                f = i / float(pts_per_side)
                x = x0 + (x1 - x0) * f
                y = y0 + (y1 - y0) * f
                
                th1, th2 = inverse_kinematics(x, y, elbow_up=ELBOW_UP)
                logger.set_cmd(th1, th2, 'square')
                move_rad(ax_s, zero_s, gear_s, th1, TH1_MIN_DEG, TH1_MAX_DEG, 'Shoulder', sign_s)
                move_rad(ax_e, zero_e, gear_e, th2, TH2_MIN_DEG, TH2_MAX_DEG, 'Elbow', sign_e)
                
                if pt_idx % (total_points // 8) == 0:
                    print(f'  loop {loop+1}/{N_LOOPS}  seg={seg+1}/4  xy=({x*100:+5.2f}, {y*100:+5.2f}) cm')
                
                pt_idx += 1
                time.sleep(dt)
                
        # Close the loop precisely at the final corner
        x, y = corners[4]
        th1, th2 = inverse_kinematics(x, y, elbow_up=ELBOW_UP)
        logger.set_cmd(th1, th2, 'square')
        move_rad(ax_s, zero_s, gear_s, th1, TH1_MIN_DEG, TH1_MAX_DEG, 'Shoulder', sign_s)
        move_rad(ax_e, zero_e, gear_e, th2, TH2_MIN_DEG, TH2_MAX_DEG, 'Elbow', sign_e)
        time.sleep(dt)


def precheck_square(cx, cy, side_length):
    print(f'\nPre-checking square:')
    print(f'  center ({cx*100:+.2f}, {cy*100:+.2f}) cm, side {side_length*100:.2f} cm')
    bad_joint, bad_ws, bad_reach = [], [], []
    worst_th2 = -999
    
    half = side_length / 2.0
    corners = [
        (cx + half, cy + half),
        (cx - half, cy + half),
        (cx - half, cy - half),
        (cx + half, cy - half),
        (cx + half, cy + half)
    ]
    pts_per_side = max(10, N_POINTS // 4)
    
    pt_idx = 0
    for seg in range(4):
        x0, y0 = corners[seg]
        x1, y1 = corners[seg+1]
        for i in range(pts_per_side):
            f = i / float(pts_per_side)
            x = x0 + (x1 - x0) * f
            y = y0 + (y1 - y0) * f
            
            try:
                th1, th2 = inverse_kinematics(x, y, elbow_up=ELBOW_UP)
            except ValueError as e:
                bad_reach.append((pt_idx, x, y, str(e))); pt_idx += 1; continue
            worst_th2 = max(worst_th2, math.degrees(th2))
            try: check_joint_limits(th1, th2)
            except ValueError as e: bad_joint.append((pt_idx, x, y, str(e)))
            try: check_workspace_bounds(x, y)
            except ValueError as e: bad_ws.append((pt_idx, x, y, str(e)))
            pt_idx += 1

    if bad_reach or bad_joint or bad_ws:
        if bad_reach:
            print(f'  ❌ {len(bad_reach)} unreachable points')
            for i,x,y,msg in bad_reach[:3]: print(f'    pt {i}: {msg}')
        if bad_joint:
            print(f'  ❌ {len(bad_joint)} joint-limit violations')
            for i,x,y,msg in bad_joint[:3]: print(f'    pt {i}: {msg}')
        if bad_ws:
            print(f'  ❌ {len(bad_ws)} workspace violations')
            for i,x,y,msg in bad_ws[:3]: print(f'    pt {i}: {msg}')
        return False
    print(f'  ✓ all {pt_idx} points OK')
    print(f'    - reachable, within joint limits (elbow margin: {TH2_MAX_DEG-worst_th2:.2f}°), inside workspace')
    return True


# ════════════════════════════════════════════════════════════
# TUNING REPORT
# ════════════════════════════════════════════════════════════
def tuning_report(samples, txt_path, cx, cy, side_length):
    import numpy as np
    if len(samples) < 10: return
    arr = np.array([(s[0],s[1],s[2],s[3],s[4]) for s in samples], dtype=float)
    phases = [s[5] for s in samples]
    t=arr[:,0]; a1=arr[:,1]; a2=arr[:,2]; c1=arr[:,3]; c2=arr[:,4]
    err1=a1-c1; err2=a2-c2
    lines = ['═'*47, '  TUNING REPORT', '═'*47,
             f'  Square: center({cx*100:+.2f}, {cy*100:+.2f}) cm  side={side_length*100:.2f} cm',
             f'  Duration: {t[-1]:.2f}s   Samples: {len(samples)}', '']
    for ph in ['move_to_start','square','return']:
        mask = np.array([p==ph for p in phases])
        if not mask.any(): continue
        r1=float(np.sqrt(np.mean(err1[mask]**2))); p1=float(np.max(np.abs(err1[mask])))
        r2=float(np.sqrt(np.mean(err2[mask]**2))); p2=float(np.max(np.abs(err2[mask])))
        lines += [f'━━ {ph} ━━',
                  f'  Shoulder: rms {r1:5.2f}°  peak {p1:5.2f}°',
                  f'  Elbow   : rms {r2:5.2f}°  peak {p2:5.2f}°', '']
    cm = np.array([p=='square' for p in phases])
    if cm.any():
        worst = max(float(np.sqrt(np.mean(err1[cm]**2))), float(np.sqrt(np.mean(err2[cm]**2))))
        verdict = '✅ Excellent' if worst<1 else '🟢 Good' if worst<2 else '🟡 Acceptable' if worst<4 else '🔴 Poor'
        lines += [f'━━ Verdict ━━', f'  {verdict} — RMS {worst:.2f}°', '═'*47]
    text = '\n'.join(lines)
    print('\n'+text)
    with open(txt_path,'w') as f: f.write(text+'\n')


# ════════════════════════════════════════════════════════════
# MAIN
# ════════════════════════════════════════════════════════════
print('═══ Square in Calibrated Workspace ═══')
print(f'  Workspace : {WORKSPACE_WIDTH*100:.1f} × {WORKSPACE_HEIGHT*100:.1f} cm '
      f'centered at ({WORKSPACE_CENTER_X*100:+.2f}, {WORKSPACE_CENTER_Y*100:+.2f}) cm')
print(f'  Max side fit: {MAX_SIDE_FIT*100:.2f} cm  |  Chosen: {SIDE_LENGTH*100:.2f} cm')
print(f'  Joint limits: θ1 [{TH1_MIN_DEG:+.2f}, {TH1_MAX_DEG:+.2f}]°, '
      f'θ2 [{TH2_MIN_DEG:+.2f}, {TH2_MAX_DEG:+.2f}]°')

if not precheck_square(CENTER_X, CENTER_Y, SIDE_LENGTH):
    print('\n❌ Square does not fit.'); sys.exit(1)

ax_s, zero_s, gear_s, odrv_s, sign_s = setup(SERIAL_SHOULDER, GEAR_SHOULDER,
                                      SHOULDER_TUNING, 'Shoulder', START_SHOULDER_DEG, sign=-1.0)
ax_e, zero_e, gear_e, odrv_e, sign_e = setup(SERIAL_ELBOW, GEAR_ELBOW,
                                      ELBOW_TUNING, 'Elbow', START_ELBOW_DEG, sign=1.0)
joints = (ax_s, zero_s, gear_s, sign_s, ax_e, zero_e, gear_e, sign_e)

INITIAL_TH1 = get_angle_rad(ax_s, zero_s, gear_s, sign_s)
INITIAL_TH2 = get_angle_rad(ax_e, zero_e, gear_e, sign_e)
init_x, init_y = forward_kinematics(INITIAL_TH1, INITIAL_TH2)
print(f'\nInitial pose: θ1={math.degrees(INITIAL_TH1):+.2f}°  θ2={math.degrees(INITIAL_TH2):+.2f}°')
print(f'  Tip: ({init_x*100:+.2f}, {init_y*100:+.2f}) cm')

# Sanity check: initial pose should be near (0.01, 62.5) cm
expected_x = 0.01
expected_y = (L1 + L2) * 100
if abs(init_x*100 - expected_x) > 5.0 or abs(init_y*100 - expected_y) > 5.0:
    print(f'\n⚠️  WARNING: Initial tip ({init_x*100:+.2f}, {init_y*100:+.2f}) cm is far from')
    print(f'   expected ({expected_x:+.2f}, {expected_y:+.2f}) cm — check physical pose and L1/L2 values')

logger = JointLogger(joints, rate_hz=LOG_RATE_HZ)
logger.set_cmd(INITIAL_TH1, INITIAL_TH2, 'idle')
logger.start()

try:
    time.sleep(0.5)
    # Start at the top-right corner of the square
    start_x = CENTER_X + (SIDE_LENGTH / 2.0)
    start_y = CENTER_Y + (SIDE_LENGTH / 2.0)
    
    print(f'\n[Phase 1/3] Move to square start ({start_x*100:+.2f}, {start_y*100:+.2f}) cm...')
    th1_0, th2_0 = inverse_kinematics(start_x, start_y, elbow_up=ELBOW_UP)
    interpolated_move(joints, logger, th1_0, th2_0, MOVE_TO_START, 'move_to_start')
    time.sleep(SETTLE_TIME)

    print(f'\n[Phase 2/3] Tracing {N_LOOPS} square(s)...')
    trace_square(joints, logger, CENTER_X, CENTER_Y, SIDE_LENGTH)
    print('  ✓ Done.')
    time.sleep(SETTLE_TIME)

except KeyboardInterrupt:
    print('\n[Ctrl-C]')
except Exception as e:
    print(f'\n⚠️ {type(e).__name__}: {e}')

finally:
    try:
        print(f'\n[Phase 3/3] Returning to initial pose...')
        interpolated_move(joints, logger, INITIAL_TH1, INITIAL_TH2, RETURN_TIME, 'return')
        time.sleep(SETTLE_TIME)
        fx, fy = forward_kinematics(get_angle_rad(ax_s, zero_s, gear_s, sign_s),
                                    get_angle_rad(ax_e, zero_e, gear_e, sign_e))
        print(f'  ✓ Tip at ({fx*100:+.2f}, {fy*100:+.2f}) cm')
    except Exception as e:
        print(f'⚠️ return failed: {e}')

    logger.set_cmd(INITIAL_TH1, INITIAL_TH2, 'idle')
    time.sleep(0.5)
    print('\nIdling motors...')
    ax_s.requested_state = AxisState.IDLE
    ax_e.requested_state = AxisState.IDLE
    logger.stop()
    logger.save_csv(LOG_CSV)
    logger.save_plot(LOG_PNG, CENTER_X*100, CENTER_Y*100, SIDE_LENGTH*100)
    tuning_report(logger.samples, TUNING_TXT, CENTER_X, CENTER_Y, SIDE_LENGTH)
    print('\nDone.')