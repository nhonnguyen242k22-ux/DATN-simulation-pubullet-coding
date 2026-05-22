"""
Chapter 5 Kinematics & Control — Complete Validation Suite
Giao diện: fk_test.py style (sliders trái + panel phải)

6 Module validate (kéo slider "Module 0-5"):
  0  FK-VALIDATE   — Analytical DH FK (Eq.5.17) vs PyBullet getLinkState
  1  IK+DECOUPLE   — Newton-Raphson IK (Eq.5.26-27) + Static Decoupler (Eq.5.8-10)
  2  TENDON-ROUTE  — Routing matrix h=R·q (Eq.5.1-5.4), coupling q3=C·q2 (Eq.5.7)
  3  CASCADE-PID   — Outer P + Inner PI + Anti-Windup (Sec.5.5), drives index_mcp
  4  SCURVE+FSM    — S-Curve trajectory (Sec.5.7.5) + 6-state FSM (Sec.5.6)
  5  SINGULARITY   — det(J) live, singular khi q1=q2=0 (Sec.5.3.8)
"""
import pybullet as p
import pybullet_data
import time, os, math

exec(open(os.path.expanduser("~/bionic_hand_sim/fix_urdf.py")).read())
URDF = os.path.expanduser("~/bionic_hand_sim/assembly_1/urdf/assembly_1_fixed.urdf")

# ── Ch.5 thesis parameters ─────────────────────────────────────────────────
L1, L2, L3 = 39.2, 30.0, 31.0   # mm — proximal / middle / distal phalanx
C           = 0.67               # coupling ratio: q3 = C·q2 (Eq.5.7)
R11, R21    = 6.0, 4.0           # tendon pulley radii at MCP (mm)
R22, R23    = 6.0, 6.0           # tendon pulley radii at PIP / DIP (mm)
RM1, RM2    = 6.0, 6.0           # motor spool radii (mm)

KP_POS      = 8.0                # outer P gain  (s⁻¹)  Eq.5.30
KP_VEL      = 0.10               # inner PI proportional  (scaled for sim)
KI_VEL      = 1.5                # inner PI integral      (scaled for sim)
MAX_VEL     = math.pi            # velocity saturation (rad/s)
PWM_MAX     = 0.90               # anti-windup threshold  Sec.5.5.4
T_HOLD_MAX  = 5.0                # hold timeout (s)       Sec.5.5.4
EMA_TAU     = 0.005              # EMA filter time constant (s)

SIM_HZ      = 240
DT          = 1.0 / SIM_HZ
EMA_ALPHA   = DT / (DT + EMA_TAU)

AXIS_LEN    = 0.010
PANEL_X     = 0.20

# ── Connect ────────────────────────────────────────────────────────────────
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.resetDebugVisualizerCamera(
    cameraDistance=0.5, cameraYaw=45,
    cameraPitch=-45, cameraTargetPosition=[0, 0, 0.1]
)
p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"))
robot_id = p.loadURDF(
    URDF, basePosition=[0, 0, 0.15],
    baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
    useFixedBase=True
)

# ── Build joint / link maps ────────────────────────────────────────────────
name_to_id = {}
link_id    = {}
for i in range(p.getNumJoints(robot_id)):
    info = p.getJointInfo(robot_id, i)
    name_to_id[info[1].decode()]  = i
    link_id[info[12].decode()]    = i

# ── Sign & range config (identical to fk_test.py) ─────────────────────────
negative_axis_joints = [
    "index_mcp_0", "little_mcp_0", "ring_mcp_0",
    "index_pip_0", "middle_pip_0", "ring_dip_0", "thumb_ip_0",
]
custom_ranges = {
    "thumb_dmc_0": (-0.5236, 2.0944, 0.0),
    "thumb_mcp_0": (-0.5236, 2.0944, 0.0),
}
finger_groups = {
    "Thumb":  ["thumb_dmc_0", "thumb_mcp_0", "thumb_ip_0"],
    "Index":  ["index_mcp_0", "index_pip_0",  "index_dip_0"],
    "Middle": ["middle_mcp_0","middle_pip_0", "middle_dip_0"],
    "Ring":   ["ring_mcp_0",  "ring_pip_0",  "ring_dip_0"],
    "Little": ["little_mcp_0","little_pip_0","little_dip_0"],
}

sliders = {}
signs   = {}
for finger, joints in finger_groups.items():
    for jname in joints:
        if jname not in name_to_id:
            continue
        jid = name_to_id[jname]
        lo, hi, default = custom_ranges.get(jname, (0.0, 1.5708, 0.0))
        sliders[jid] = p.addUserDebugParameter(
            f"{finger} {jname.split('_')[1].upper()}", lo, hi, default
        )
        signs[jid] = -1 if jname in negative_axis_joints else 1

# ── Fingertip tracking (same 5 tips as fk_test.py) ────────────────────────
TIPS = [
    ("thumb_dp_2_1", "THUMB",  [1.0, 0.55, 0.0]),
    ("dp_10_3_1",    "INDEX",  [0.2, 1.0,  0.2]),
    ("dp_10_3_3",    "MIDDLE", [0.2, 0.6,  1.0]),
    ("dp_10_3_2",    "RING",   [1.0, 0.2,  1.0]),
    ("dp_10_3",      "LITTLE", [1.0, 1.0,  0.2]),
]
tip_data = []
for idx, (lname, label, color) in enumerate(TIPS):
    lid = link_id.get(lname)
    if lid is None:
        continue
    pos = p.getLinkState(robot_id, lid)[4]
    pp  = [PANEL_X, 0.0, 0.46 - idx * 0.055]
    pid = p.addUserDebugText(
        f"{label}: ({pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f})",
        pp, textColorRGB=color, textSize=0.85
    )
    lx = p.addUserDebugLine(pos, [pos[0]+AXIS_LEN,pos[1],pos[2]], [1,0,0], 3)
    ly = p.addUserDebugLine(pos, [pos[0],pos[1]+AXIS_LEN,pos[2]], [0,1,0], 3)
    lz = p.addUserDebugLine(pos, [pos[0],pos[1],pos[2]+AXIS_LEN], [0,0,1], 3)
    tip_data.append({"lid":lid,"label":label,"color":color,
                     "panel_id":pid,"panel_pos":pp,"lx":lx,"ly":ly,"lz":lz})

# ── Module selector + module-specific controls ─────────────────────────────
p.addUserDebugText("=== Ch.5 VALIDATION ===",
                   [PANEL_X, 0.0, 0.54], textColorRGB=[1,1,0], textSize=0.9)

MODULE_NAMES = [
    "0:FK-VALIDATE",
    "1:IK+DECOUPLE",
    "2:TENDON-ROUTE",
    "3:CASCADE-PID",
    "4:SCURVE+FSM",
    "5:SINGULARITY",
]
module_slider  = p.addUserDebugParameter("Module (0-5)", 0, 5, 0)
ik_slider_x    = p.addUserDebugParameter("IK/PID Target X mm (mod 1)", 0, 95, 50)
ik_slider_z    = p.addUserDebugParameter("IK Target Z mm (mod 1)",      0,120, 80)
pid_tgt_slider = p.addUserDebugParameter("PID/FSM Target q1 deg (3,4)", 0, 88, 40)

# ── Algorithm output panel (8 rows, below tip panel) ──────────────────────
ALGO_Y0  = 0.20
ALGO_DY  = 0.045
ALGO_ROWS = 9
algo_ids = []
for i in range(ALGO_ROWS):
    tid = p.addUserDebugText(
        "", [PANEL_X, 0.0, ALGO_Y0 - i * ALGO_DY],
        textColorRGB=[1, 1, 1], textSize=0.78
    )
    algo_ids.append(tid)

def set_panel(lines):
    for i, tid in enumerate(algo_ids):
        txt = lines[i] if i < len(lines) else ""
        p.addUserDebugText(
            txt, [PANEL_X, 0.0, ALGO_Y0 - i * ALGO_DY],
            textColorRGB=[1, 1, 1], textSize=0.78, replaceItemUniqueId=tid
        )

# ══════════════════════════════════════════════════════════════════════════
#  ALGORITHM IMPLEMENTATIONS
# ══════════════════════════════════════════════════════════════════════════

# ── MODULE 0 / 1: Analytical FK (Eq.5.17) ─────────────────────────────────
def fk_local(q1_rad, q2_rad):
    """
    Planar 3-link FK in local MCP frame (a2=0).
    Returns (X_mm, Z_mm):  X = dorsal reach,  Z = distal reach from MCP.
    q3 = C·q2 embedded (underactuated coupling, Eq.5.7).
    """
    q3 = C * q2_rad
    x = (L1 * math.sin(q1_rad)
         + L2 * math.sin(q1_rad + q2_rad)
         + L3 * math.sin(q1_rad + q2_rad + q3))
    z = (L1 * math.cos(q1_rad)
         + L2 * math.cos(q1_rad + q2_rad)
         + L3 * math.cos(q1_rad + q2_rad + q3))
    return x, z

# ── MODULE 1: Analytical Jacobian (Eq.5.27) ───────────────────────────────
def jacobian(q1, q2):
    """2×2 analytical Jacobian for Phase-1 IK."""
    q12  = q1 + q2
    q123 = q1 + q2 + C * q2          # = q1 + (1+C)·q2

    df1dq1 =  L1*math.cos(q1)  + L2*math.cos(q12)  + L3*math.cos(q123)
    df1dq2 =  L2*math.cos(q12) + L3*(1+C)*math.cos(q123)
    df2dq1 = -L1*math.sin(q1)  - L2*math.sin(q12)  - L3*math.sin(q123)
    df2dq2 = -L2*math.sin(q12) - L3*(1+C)*math.sin(q123)
    return [[df1dq1, df1dq2],
            [df2dq1, df2dq2]]

def det2(J):
    return J[0][0]*J[1][1] - J[0][1]*J[1][0]

def inv2(J):
    d = det2(J)
    if abs(d) < 1e-6:
        return None
    return [[ J[1][1]/d, -J[0][1]/d],
            [-J[1][0]/d,  J[0][0]/d]]

def newton_raphson_ik(xd, zd, q1_init=0.4, q2_init=0.6,
                      max_iter=150, tol=0.05):
    """
    Newton-Raphson IK Phase-1 (Eq.5.26).
    Returns (q1, q2, residual_mm, converged).
    xd, zd in mm relative to MCP.
    """
    q1, q2 = q1_init, q2_init
    for _ in range(max_iter):
        x, z = fk_local(q1, q2)
        f1, f2 = x - xd, z - zd
        res = math.sqrt(f1*f1 + f2*f2)
        if res < tol:
            return q1, q2, res, True
        J  = jacobian(q1, q2)
        Ji = inv2(J)
        if Ji is None:
            break
        dq1 = Ji[0][0]*f1 + Ji[0][1]*f2
        dq2 = Ji[1][0]*f1 + Ji[1][1]*f2
        q1 = max(0.0, min(math.pi/2, q1 - dq1))
        q2 = max(0.0, min(math.pi/2, q2 - dq2))
    x, z = fk_local(q1, q2)
    return q1, q2, math.sqrt((x-xd)**2+(z-zd)**2), False

# ── MODULE 1: Static Decoupler (Eq.5.8–5.10) ──────────────────────────────
def decoupler(u1_rad, u2_rad):
    """
    Kinematic Decoupler: maps joint commands → motor references.
    Motor 1 drives MCP; Motor 2 drives PIP+DIP via coupling.
    """
    tm1 = (R11 / RM1) * u1_rad
    tm2 = (R21 / RM2) * u1_rad + ((R22 + C * R23) / RM2) * u2_rad
    return tm1, tm2

# ── MODULE 2: Tendon routing matrix (Eq.5.1–5.4) ──────────────────────────
def tendon_routing(q1_rad, q2_rad):
    """
    h = R_tendon · q   (q3 = C·q2 substituted).
    Returns (h1_mm, h2_mm) — cable length changes.
    """
    q3  = C * q2_rad
    h1  = R11 * q1_rad
    h2  = R21 * q1_rad + R22 * q2_rad + R23 * q3
    return h1, h2

# ── MODULE 3: Cascade PID state ────────────────────────────────────────────
pid = {
    "I_vel":        0.0,
    "vel_filtered": 0.0,
    "prev_angle":   0.0,
    "sat_timer":    0.0,
    "pwm":          0.0,
}

def cascade_pid_step(q_ref, q_actual, dt):
    """
    One PID ISR tick (Sec.5.5).
    Returns (pwm, e_pos_deg, vel_ref_dps, vel_act_dps, e_vel_dps, I_vel, aw).
    """
    # Outer P loop (Eq.5.29)
    e_pos = q_ref - q_actual
    if abs(e_pos) < math.radians(0.5):
        e_pos = 0.0
    vel_ref = max(-MAX_VEL, min(MAX_VEL, KP_POS * e_pos))

    # EMA velocity estimate (Eq.5.35)
    vel_raw = (q_actual - pid["prev_angle"]) / dt
    pid["vel_filtered"] = EMA_ALPHA * vel_raw + (1.0 - EMA_ALPHA) * pid["vel_filtered"]
    pid["prev_angle"] = q_actual

    # Inner PI loop (Eq.5.31–5.34)
    e_vel   = vel_ref - pid["vel_filtered"]
    pwm_raw = KP_VEL * e_vel + pid["I_vel"]

    # Anti-windup clamp (Sec.5.5.4)
    if abs(pwm_raw) >= PWM_MAX:
        pid["sat_timer"] += dt          # freeze integrator
        aw = True
    else:
        pid["I_vel"] += KI_VEL * e_vel * dt
        pid["sat_timer"] = 0.0
        aw = False

    pwm = max(-1.0, min(1.0, pwm_raw))
    if pid["sat_timer"] > T_HOLD_MAX:   # hold mode: 30%
        pwm = math.copysign(0.30, pwm_raw)

    pid["pwm"] = pwm
    return (pwm,
            math.degrees(e_pos), math.degrees(vel_ref),
            math.degrees(pid["vel_filtered"]), math.degrees(e_vel),
            pid["I_vel"], aw)

# ── MODULE 4: S-Curve trajectory (Sec.5.7.5) ──────────────────────────────
class SCurve:
    def __init__(self):
        self.q_ref = 0.0
        self.q_tgt = 0.0
        self.done  = True
        self.phase = "DONE"           # for display

    def set_target(self, q_cur, q_tgt):
        self.q_ref = q_cur
        self.q_tgt = q_tgt
        self.done  = abs(q_tgt - q_cur) < 1e-4
        self.phase = "DONE" if self.done else "MOVING"

    def step(self, dt, v_max=MAX_VEL * 0.4):
        if self.done:
            return self.q_ref
        remain = self.q_tgt - self.q_ref
        step   = math.copysign(min(v_max * dt, abs(remain)), remain)
        self.q_ref += step
        if abs(self.q_tgt - self.q_ref) < 1e-4:
            self.q_ref = self.q_tgt
            self.done  = True
            self.phase = "DONE"
        return self.q_ref

scurve = SCurve()

# ── MODULE 4: FSM (Sec.5.6) ────────────────────────────────────────────────
FSM_NAMES = {
    0:"S0:INIT", 1:"S1:HOMING", 2:"S2:IDLE",
    3:"S3:APPROACH", 4:"S4:GRASPING", 5:"S5:E-STOP"
}
fsm_state  = 2     # start at IDLE (sim already homed)
fsm_timer  = 0.0

def fsm_tick(q_actual, q_tgt, pwm_sat, dt):
    """Advance FSM one step. Returns current state name + event description."""
    global fsm_state, fsm_timer
    fsm_timer += dt
    event = ""
    # Transition table (Table 5.6)
    if fsm_state == 2:
        if abs(q_tgt - q_actual) > math.radians(3):
            fsm_state = 3; fsm_timer = 0.0
            event = "T2: IDLE→APPROACH (target set)"
    elif fsm_state == 3:
        if abs(q_tgt - q_actual) < math.radians(2):
            fsm_state = 4; fsm_timer = 0.0
            event = "T3a: APPROACH→GRASP (pos reached)"
        elif pwm_sat:
            fsm_state = 4; fsm_timer = 0.0
            event = "T3b: APPROACH→GRASP (PWM sat)"
    elif fsm_state == 4:
        if abs(q_tgt - q_actual) < math.radians(2) and fsm_timer > 1.0:
            fsm_state = 2; fsm_timer = 0.0
            event = "T4: GRASP→IDLE (release)"
    if event:
        print(f"  [FSM] {event}")
    return FSM_NAMES[fsm_state], event

# ══════════════════════════════════════════════════════════════════════════
#  IDs for index finger joints used in modules 0-5
# ══════════════════════════════════════════════════════════════════════════
idx_mcp_jid  = name_to_id.get("index_mcp_0")   # joint id
idx_pip_jid  = name_to_id.get("index_pip_0")
idx_dip_jid  = name_to_id.get("index_dip_0")
idx_tip_lid  = link_id.get("dp_10_3_1")         # distal phalanx link

# ── Print startup summary ──────────────────────────────────────────────────
print("\n" + "="*60)
print("  Ch.5 Kinematics & Control — Validation Suite")
print("="*60)
print("  Kéo slider 'Module (0-5)' để chọn module cần validate.")
print()
print("  0  FK-VALIDATE   Analytical DH (Eq.5.17) vs PyBullet")
print("  1  IK+DECOUPLE   Newton-Raphson (Eq.5.26) + Decoupler (Eq.5.8)")
print("  2  TENDON-ROUTE  h=R·q (Eq.5.1) + coupling q3=C·q2 (Eq.5.7)")
print("  3  CASCADE-PID   Outer P + Inner PI + Anti-Windup (Sec.5.5)")
print("  4  SCURVE+FSM    Trajectory (Sec.5.7.5) + 6-state FSM (Sec.5.6)")
print("  5  SINGULARITY   det(J) live (Sec.5.3.8)")
print("="*60)
print()

# ── Main loop state ────────────────────────────────────────────────────────
loop_count   = 0
prev_module  = -1
prev_pid_tgt = -999.0
pid_q_ref    = 0.0
scurve.set_target(0.0, 0.0)

try:
    while True:
        loop_count += 1
        module = int(round(p.readUserDebugParameter(module_slider)))

        # ── Read index finger slider values (flexion angles, always positive) ─
        q1_rad = p.readUserDebugParameter(sliders[idx_mcp_jid])   # MCP flexion
        q2_rad = p.readUserDebugParameter(sliders[idx_pip_jid])   # PIP flexion
        q3_rad = C * q2_rad                                        # DIP (coupling)

        # ── Drive all joints from sliders; module 3/4 overrides index_mcp ────
        for jid, sid in sliders.items():
            if module in (3, 4) and jid == idx_mcp_jid:
                continue    # PID will take over MCP
            val = p.readUserDebugParameter(sid)
            p.setJointMotorControl2(robot_id, jid, p.POSITION_CONTROL,
                                    targetPosition=val * signs[jid], force=5.0)

        # ── Update fingertip FK display (identical to fk_test.py) ─────────────
        for td in tip_data:
            pos = p.getLinkState(robot_id, td["lid"])[4]
            p.addUserDebugText(
                f"{td['label']}: ({pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f})",
                td["panel_pos"], textColorRGB=td["color"], textSize=0.85,
                replaceItemUniqueId=td["panel_id"]
            )
            p.addUserDebugLine(pos,[pos[0]+AXIS_LEN,pos[1],pos[2]],
                               [1,0,0],3,replaceItemUniqueId=td["lx"])
            p.addUserDebugLine(pos,[pos[0],pos[1]+AXIS_LEN,pos[2]],
                               [0,1,0],3,replaceItemUniqueId=td["ly"])
            p.addUserDebugLine(pos,[pos[0],pos[1],pos[2]+AXIS_LEN],
                               [0,0,1],3,replaceItemUniqueId=td["lz"])

        # ════════════════════════════════════════════════════════════════
        #  MODULE DISPATCH
        # ════════════════════════════════════════════════════════════════

        # ── Module 0: FK VALIDATION ──────────────────────────────────────
        if module == 0:
            x_an, z_an = fk_local(q1_rad, q2_rad)
            reach_an   = math.sqrt(x_an**2 + z_an**2)

            # PyBullet: tip position relative to MCP joint origin
            tip_pos = p.getLinkState(robot_id, idx_tip_lid)[4]
            mcp_pos = p.getLinkState(robot_id, idx_mcp_jid)[4]
            dx = tip_pos[0] - mcp_pos[0]
            dy = tip_pos[1] - mcp_pos[1]
            dz = tip_pos[2] - mcp_pos[2]
            reach_pb = math.sqrt(dx*dx + dy*dy + dz*dz) * 1000   # → mm

            err_mm = abs(reach_an - reach_pb)

            # Numerical check from thesis (q1=30°, q2=50°)
            x_ref, z_ref = fk_local(math.radians(30), math.radians(50))

            lines = [
                f"=0= FK VALIDATION (Eq.5.16-5.17)",
                f"q1={math.degrees(q1_rad):.1f}d  q2={math.degrees(q2_rad):.1f}d"
                f"  q3=C·q2={math.degrees(q3_rad):.1f}d",
                f"[Analytic Eq.5.17]",
                f"  X={x_an:+.2f}mm  Z={z_an:+.2f}mm",
                f"  Reach_local={reach_an:.2f}mm",
                f"[PyBullet getLinkState]",
                f"  Reach_3D={reach_pb:.2f}mm  Err={err_mm:.2f}mm",
                f"[Thesis num. check q1=30 q2=50]",
                f"  X={x_ref:.2f}mm(th:77.58)  Z={z_ref:.2f}mm(th:26.80)",
            ]

        # ── Module 1: IK + DECOUPLER ─────────────────────────────────────
        elif module == 1:
            xd = p.readUserDebugParameter(ik_slider_x)
            zd = p.readUserDebugParameter(ik_slider_z)

            q1_ik, q2_ik, res, conv = newton_raphson_ik(xd, zd)
            q3_ik = C * q2_ik

            # FK forward-check
            x_chk, z_chk = fk_local(q1_ik, q2_ik)

            # Static Decoupler
            tm1, tm2 = decoupler(q1_ik, q2_ik)

            # Cross-coupling verification: Δtm2 caused by Δq1 only
            tm1_dq1, tm2_dq1 = decoupler(q1_ik + 0.1, q2_ik)
            coupling_comp = (tm2_dq1 - tm2) - (R21/RM2) * 0.1   # should ≈ 0

            status = "CONVERGED" if conv else "NOT CONV"
            lines = [
                f"=1= IK + STATIC DECOUPLER",
                f"Target: X={xd:.1f}mm  Z={zd:.1f}mm",
                f"[Newton-Raphson IK (Eq.5.26-27)]  {status}",
                f"  q1={math.degrees(q1_ik):.2f}d  q2={math.degrees(q2_ik):.2f}d"
                f"  q3={math.degrees(q3_ik):.2f}d",
                f"  Residual={res:.3f}mm  FK-check X={x_chk:.1f} Z={z_chk:.1f}",
                f"[Decoupler (Eq.5.8-5.10)]",
                f"  tm1={math.degrees(tm1):.2f}d  tm2={math.degrees(tm2):.2f}d",
                f"  Cross-coup comp error={coupling_comp*1000:.4f}e-3 (→0 OK)",
                f"  k_decouple=R21/RM2={R21/RM2:.3f}",
            ]

        # ── Module 2: TENDON ROUTING ─────────────────────────────────────
        elif module == 2:
            h1, h2 = tendon_routing(q1_rad, q2_rad)

            # Verify coupling: compare slider DIP vs coupling prediction
            dip_actual_pb = abs(p.getJointState(robot_id, idx_dip_jid)[0])
            q3_pred       = C * q2_rad
            coupling_err  = math.degrees(abs(dip_actual_pb - q3_pred))

            # Torque distribution (Eq.5.4), T=[1,1] N for illustration
            T1, T2 = 1.0, 1.0
            tau1 = R11*T1 + R21*T2
            tau2 = R22*T2
            tau3 = R23*T2

            lines = [
                f"=2= TENDON ROUTING MATRIX (Eq.5.1-5.7)",
                f"q1={math.degrees(q1_rad):.1f}d  q2={math.degrees(q2_rad):.1f}d"
                f"  C={C}",
                f"[h = R_tendon · q  (Eq.5.2)]",
                f"  h1=r11·q1={R11}·{q1_rad:.3f}={h1:.3f}mm",
                f"  h2=r21·q1+r22·q2+r23·q3={h2:.3f}mm",
                f"[Coupling q3=C·q2 (Eq.5.7)]",
                f"  Predicted={math.degrees(q3_pred):.2f}d"
                f"  PyBullet={math.degrees(dip_actual_pb):.2f}d"
                f"  Err={coupling_err:.2f}d",
                f"[Torque R^T·T (Eq.5.4) @ T=[1,1]N]",
                f"  τ1={tau1:.1f}  τ2={tau2:.1f}  τ3={tau3:.1f} N·mm",
            ]

        # ── Module 3: CASCADE PID ─────────────────────────────────────────
        elif module == 3:
            q_tgt_deg = p.readUserDebugParameter(pid_tgt_slider)
            q_tgt     = math.radians(q_tgt_deg)

            if abs(q_tgt - prev_pid_tgt) > math.radians(1):
                q_cur = abs(p.getJointState(robot_id, idx_mcp_jid)[0])
                scurve.set_target(q_cur, q_tgt)
                pid["I_vel"] = 0.0
                pid["sat_timer"] = 0.0
                prev_pid_tgt = q_tgt

            pid_q_ref = scurve.step(DT)
            # q_actual: index_mcp is negative_axis → negate PyBullet reading
            q_actual  = -p.getJointState(robot_id, idx_mcp_jid)[0]
            q_actual  = max(0.0, q_actual)

            pwm, ep, vr, va, ev, iv, aw = cascade_pid_step(pid_q_ref, q_actual, DT)

            # Apply via VELOCITY_CONTROL (sign: negative_axis flip)
            vel_cmd = pwm * MAX_VEL
            p.setJointMotorControl2(
                robot_id, idx_mcp_jid, p.VELOCITY_CONTROL,
                targetVelocity=-vel_cmd, force=8.0
            )

            aw_str = "AW-CLAMP" if aw else "OK"
            hold   = "HOLD-30%" if pid["sat_timer"] > T_HOLD_MAX else ""
            lines = [
                f"=3= CASCADE PID (Sec.5.5)",
                f"q_tgt={q_tgt_deg:.1f}d  q_ref={math.degrees(pid_q_ref):.1f}d"
                f"  q_act={math.degrees(q_actual):.1f}d",
                f"[Outer P  Kp={KP_POS}  (Eq.5.29)]",
                f"  e_pos={ep:.2f}d  vel_ref={vr:.1f}d/s",
                f"[Inner PI  Kp={KP_VEL}  Ki={KI_VEL}  (Eq.5.31-34)]",
                f"  vel_act={va:.1f}d/s  e_vel={ev:.2f}d/s",
                f"  I_vel={iv:.4f}  PWM={pwm*100:.1f}%  [{aw_str}]{hold}",
                f"[Anti-Windup  PWM_max={PWM_MAX*100:.0f}%  (Sec.5.5.4)]",
                f"  sat_timer={pid['sat_timer']:.2f}s / T_hold={T_HOLD_MAX}s",
            ]

        # ── Module 4: S-CURVE + FSM ────────────────────────────────────────
        elif module == 4:
            q_tgt_deg = p.readUserDebugParameter(pid_tgt_slider)
            q_tgt     = math.radians(q_tgt_deg)

            if abs(q_tgt - prev_pid_tgt) > math.radians(1):
                q_cur = -p.getJointState(robot_id, idx_mcp_jid)[0]
                q_cur = max(0.0, q_cur)
                scurve.set_target(q_cur, q_tgt)
                pid["I_vel"] = 0.0
                pid["sat_timer"] = 0.0
                prev_pid_tgt = q_tgt

            pid_q_ref = scurve.step(DT)
            q_actual  = max(0.0, -p.getJointState(robot_id, idx_mcp_jid)[0])

            pwm, ep, vr, va, ev, iv, aw = cascade_pid_step(pid_q_ref, q_actual, DT)
            vel_cmd = pwm * MAX_VEL
            p.setJointMotorControl2(
                robot_id, idx_mcp_jid, p.VELOCITY_CONTROL,
                targetVelocity=-vel_cmd, force=8.0
            )

            fsm_name, fsm_event = fsm_tick(q_actual, q_tgt, aw, DT)

            lines = [
                f"=4= S-CURVE + FSM (Sec.5.6-5.7.5)",
                f"[S-Curve Trajectory  v_max={MAX_VEL*0.4:.2f}rad/s]",
                f"  q_tgt={math.degrees(q_tgt):.1f}d"
                f"  q_ref={math.degrees(pid_q_ref):.1f}d  [{scurve.phase}]",
                f"  q_act={math.degrees(q_actual):.1f}d"
                f"  err={abs(math.degrees(q_tgt-q_actual)):.2f}d",
                f"[FSM (6 states, Table 5.5-5.6)]",
                f"  State: {fsm_name}",
                f"  Timer: {fsm_timer:.1f}s",
                f"  PWM={pwm*100:.1f}%  AW={'YES' if aw else 'NO'}",
                f"  Last event: {fsm_event if fsm_event else '—'}",
            ]

        # ── Module 5: SINGULARITY ANALYSIS ────────────────────────────────
        elif module == 5:
            J   = jacobian(q1_rad, q2_rad)
            d   = det2(J)
            x_t, z_t = fk_local(q1_rad, q2_rad)
            reach = math.sqrt(x_t**2 + z_t**2)
            max_r = L1 + L2 + L3   # 100.2 mm — full extension

            # Singular at q1=q2=0 (full extension)
            if abs(d) < 200:
                status = "!!! SINGULAR ZONE !!!"
                color_warn = True
            elif abs(d) < 1000:
                status = "  (near singular)"
                color_warn = False
            else:
                status = "  OK — nonsingular"
                color_warn = False

            # Numerical check from thesis (q1=q2=0)
            J_ext = jacobian(0, 0)
            d_ext = det2(J_ext)

            lines = [
                f"=5= SINGULARITY ANALYSIS (Sec.5.3.8)",
                f"q1={math.degrees(q1_rad):.1f}d  q2={math.degrees(q2_rad):.1f}d",
                f"[Jacobian J (Eq.5.27)]",
                f"  J11={J[0][0]:+.1f}  J12={J[0][1]:+.1f}",
                f"  J21={J[1][0]:+.1f}  J22={J[1][1]:+.1f}",
                f"[det(J) — zero = singular]",
                f"  det(J) = {d:+.1f} mm^2  {status}",
                f"  Reach={reach:.1f}/{max_r:.1f}mm ({reach/max_r*100:.0f}% extended)",
                f"  @ q1=q2=0: det={d_ext:.4f}  (thesis: 0.0000)",
            ]

        else:
            lines = [f"Module {module} không hợp lệ"]

        set_panel(lines)

        # ── Module switch notification ─────────────────────────────────────
        if module != prev_module:
            print(f"\n{'─'*55}")
            print(f"  >>> Module {MODULE_NAMES[module]}")
            print(f"{'─'*55}")
            prev_module = module
            # reset PID when switching away/into module 3/4
            pid["I_vel"] = 0.0
            pid["sat_timer"] = 0.0
            prev_pid_tgt = -999.0

        # ── Console print every 3 s ────────────────────────────────────────
        if loop_count % (SIM_HZ * 3) == 0:
            summary = " | ".join(l for l in lines[1:3] if l)
            print(f"  [{MODULE_NAMES[module]}]  {summary}")

        p.stepSimulation()
        time.sleep(DT)

except KeyboardInterrupt:
    p.disconnect()
    print("\n🔴 Dừng.")
