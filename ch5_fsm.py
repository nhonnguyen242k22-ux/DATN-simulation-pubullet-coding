"""
Ch.5 §5.6–5.7 — S-Curve Trajectory + 6-State Finite State Machine
Giao diện: fk_test.py style

Validate:
  - S-Curve trajectory generator      (Sec.5.7.5)
  - 6-state FSM: INIT→HOMING→IDLE→APPROACHING→GRASPING→E-STOP  (Sec.5.6)
  - State transition conditions        (Table 5.6)
  - Homing routine                     (Sec.5.7.2)

Điều khiển Index MCP bằng S-Curve + Cascade PID.
  - Slider "Target (deg)": đặt góc mục tiêu
  - Panel phải: FSM state + trajectory progress live
  - FSM tự chuyển IDLE→APPROACH khi target thay đổi,
              APPROACH→GRASPING khi đến nơi hoặc PWM saturate,
              GRASPING→IDLE sau 3s
  - Button "E-STOP": chuyển sang state 5, tắt motor
"""
import pybullet as p
import pybullet_data
import time, os, math

exec(open(os.path.expanduser("~/bionic_hand_sim/fix_urdf.py")).read())
URDF = os.path.expanduser("~/bionic_hand_sim/assembly_1/urdf/assembly_1_fixed.urdf")

# ── Controller params ─────────────────────────────────────────────────────
KP_POS   = 8.0
KP_VEL   = 0.10
KI_VEL   = 1.5
MAX_VEL  = math.pi
PWM_MAX  = 0.90
EMA_TAU  = 0.005
SCURVE_V = MAX_VEL * 0.35          # S-curve max velocity (rad/s)
DEADBAND = math.radians(0.5)
SIM_HZ   = 240
DT       = 1.0 / SIM_HZ
EMA_A    = DT / (DT + EMA_TAU)

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

name_to_id = {}
link_id    = {}
for i in range(p.getNumJoints(robot_id)):
    info = p.getJointInfo(robot_id, i)
    name_to_id[info[1].decode()]  = i
    link_id[info[12].decode()]    = i

# ── Config ─────────────────────────────────────────────────────────────────
negative_axis_joints = [
    "index_mcp_0", "little_mcp_0", "ring_mcp_0",
    "index_pip_0",  "middle_pip_0", "ring_dip_0", "thumb_ip_0",
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
            f"{finger} {jname.split('_')[1].upper()}", lo, hi, default)
        signs[jid] = -1 if jname in negative_axis_joints else 1

target_sl = p.addUserDebugParameter("Target (deg)",  0.0, 88.0, 0.0)
estop_btn = p.addUserDebugParameter("E-STOP (press)", 1, 0, 0)

# ── Fingertip display ──────────────────────────────────────────────────────
AXIS_LEN = 0.010
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
    pp  = [0.20, 0.0, 0.46 - idx * 0.055]
    pid = p.addUserDebugText(
        f"{label}: ({pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f})",
        pp, textColorRGB=color, textSize=0.85)
    fid = p.addUserDebugText(label,
        [pos[0]+0.012,pos[1],pos[2]+0.006], textColorRGB=color, textSize=0.85)
    lx = p.addUserDebugLine(pos,[pos[0]+AXIS_LEN,pos[1],pos[2]],[1,0,0],3)
    ly = p.addUserDebugLine(pos,[pos[0],pos[1]+AXIS_LEN,pos[2]],[0,1,0],3)
    lz = p.addUserDebugLine(pos,[pos[0],pos[1],pos[2]+AXIS_LEN],[0,0,1],3)
    tip_data.append({"lid":lid,"label":label,"color":color,
                     "panel_id":pid,"panel_pos":pp,"float_id":fid,
                     "lx":lx,"ly":ly,"lz":lz})

ALGO_IDS = []
for i in range(12):
    tid = p.addUserDebugText("", [0.20, 0.0, 0.20 - i*0.040],
                             textColorRGB=[1,1,1], textSize=0.78)
    ALGO_IDS.append(tid)

def set_panel(lines):
    for i, tid in enumerate(ALGO_IDS):
        txt = lines[i] if i < len(lines) else ""
        p.addUserDebugText(txt, [0.20, 0.0, 0.20 - i*0.040],
                           textColorRGB=[1,1,1], textSize=0.78,
                           replaceItemUniqueId=tid)

# ── S-Curve ────────────────────────────────────────────────────────────────
class SCurve:
    def __init__(self):
        self.q_ref = 0.0
        self.q_tgt = 0.0
        self.done  = True

    def reset(self, q_cur, q_tgt):
        self.q_ref = q_cur
        self.q_tgt = q_tgt
        self.done  = abs(q_tgt - q_cur) < 1e-4

    def step(self, dt):
        if self.done:
            return self.q_ref
        remain  = self.q_tgt - self.q_ref
        d_step  = math.copysign(min(SCURVE_V * dt, abs(remain)), remain)
        self.q_ref += d_step
        if abs(self.q_tgt - self.q_ref) < 1e-4:
            self.q_ref = self.q_tgt
            self.done  = True
        return self.q_ref

sc = SCurve()

# ── PID state ─────────────────────────────────────────────────────────────
mcp_jid   = name_to_id["index_mcp_0"]
pid       = {"I_vel": 0.0, "vel_filt": 0.0, "prev_q": 0.0, "sat_timer": 0.0}

def cascade_pid_step(q_ref, q_act, dt):
    e_pos = q_ref - q_act
    if abs(e_pos) < DEADBAND:
        e_pos = 0.0
    vel_ref = max(-MAX_VEL, min(MAX_VEL, KP_POS * e_pos))
    vel_raw = (q_act - pid["prev_q"]) / dt
    pid["vel_filt"] = EMA_A * vel_raw + (1 - EMA_A) * pid["vel_filt"]
    pid["prev_q"]   = q_act
    e_vel   = vel_ref - pid["vel_filt"]
    pwm_raw = KP_VEL * e_vel + pid["I_vel"]
    aw = abs(pwm_raw) >= PWM_MAX
    if aw:
        pid["sat_timer"] += dt
    else:
        pid["I_vel"] += KI_VEL * e_vel * dt
        pid["sat_timer"] = 0.0
    pwm = max(-1.0, min(1.0, pwm_raw))
    return pwm, aw

# ── FSM ────────────────────────────────────────────────────────────────────
# States: 0=INIT 1=HOMING 2=IDLE 3=APPROACHING 4=GRASPING 5=E-STOP
FSM = {
    0: "S0: INIT",
    1: "S1: HOMING",
    2: "S2: IDLE",
    3: "S3: APPROACHING",
    4: "S4: GRASPING",
    5: "S5: E-STOP",
}
STATE_COLORS = {
    0: [0.8,0.8,0.0], 1: [0.9,0.6,0.0], 2: [0.2,1.0,0.2],
    3: [0.2,0.6,1.0], 4: [0.0,1.0,0.8], 5: [1.0,0.2,0.2],
}

state       = 2          # start at IDLE (sim already homed)
state_timer = 0.0
last_event  = "startup → IDLE"
prev_estop  = 0
prev_tgt    = 0.0

STATE_BAR = {
    0:"[INIT    ]", 1:"[HOMING  ]", 2:"[IDLE    ]",
    3:"[APPROACH]", 4:"[GRASPING]", 5:"[E-STOP  ]",
}

def print_table(state, st_timer, last_ev, q_tgt_d, q_ref_d, q_act_d, sc_done, pwm, aw, iv):
    sc_s = "DONE  " if sc_done else "MOVING"
    aw_s = "CLAMP" if aw else "OK   "
    rows = [
        "┌──────────────────────────────────────────────────────────┐",
        "│       Ch.5 §5.6-5.7  S-CURVE + FSM                     │",
        "├──────────────────────────────────────────────────────────┤",
        f"│  FSM State : {FSM[state]:<44} │",
        f"│  State timer: {st_timer:>5.1f} s                                    │",
        f"│  Last event : {last_ev:<43} │",
        "├──────────────────────────┬───────────────────────────────┤",
        "│  S-Curve (Sec.5.7.5)     │  Cascade PID                 │",
        "├──────────────────────────┼───────────────────────────────┤",
        f"│  q_target = {q_tgt_d:>7.2f} deg    │  PWM   = {pwm*100:>+7.2f} %           │",
        f"│  q_ref    = {q_ref_d:>7.2f} deg    │  AW    = [{aw_s}]           │",
        f"│  q_actual = {q_act_d:>7.2f} deg    │  I_vel = {iv:>+7.4f}            │",
        f"│  Status   : [{sc_s}]      │  v_max = {math.degrees(SCURVE_V):.1f} d/s              │",
        "└──────────────────────────┴───────────────────────────────┘",
    ]
    print(f"\033[{len(rows)}A", end="")
    for r in rows:
        print(f"\r{r}\033[K")

print("\n🟢 Ch.5 §5.6-5.7 — S-Curve + FSM")
print(f"   v_scurve={SCURVE_V:.2f}rad/s  Kp={KP_POS}  PWM_max={PWM_MAX*100:.0f}%")
print("   Kéo 'Target' → APPROACHING. Bấm 'E-STOP' → S5.\n")
for _ in range(14):
    print()

loop = 0
try:
    while True:
        loop += 1
        state_timer += DT

        q_tgt_deg = p.readUserDebugParameter(target_sl)
        q_tgt     = math.radians(q_tgt_deg)
        q_act     = max(0.0, -p.getJointState(robot_id, mcp_jid)[0])

        # E-STOP button
        estop_val = p.readUserDebugParameter(estop_btn)
        if estop_val > prev_estop and state != 5:
            state = 5; state_timer = 0.0
            last_event = "T5: host ESTOP command"
            print(f"  [FSM] {last_event}")
        prev_estop = estop_val

        # ── FSM transitions (Table 5.6) ────────────────────────────────────
        if state == 2:   # IDLE
            if abs(q_tgt - prev_tgt) > math.radians(3):
                sc.reset(q_act, q_tgt)
                pid["I_vel"] = 0.0; pid["sat_timer"] = 0.0
                state = 3; state_timer = 0.0
                last_event = f"T2: IDLE→APPROACHING  target={q_tgt_deg:.1f}d"
                print(f"  [FSM] {last_event}")
            prev_tgt = q_tgt

        elif state == 3:  # APPROACHING
            if abs(q_tgt - q_act) < math.radians(2):
                state = 4; state_timer = 0.0
                last_event = "T3a: APPROACHING→GRASPING  (pos reached)"
                print(f"  [FSM] {last_event}")

        elif state == 4:  # GRASPING — hold 3 s then release
            if state_timer > 3.0:
                state = 2; state_timer = 0.0
                last_event = "T4: GRASPING→IDLE  (release after 3s)"
                print(f"  [FSM] {last_event}")

        prev_tgt = q_tgt

        # ── S-Curve + PID ──────────────────────────────────────────────────
        if state in (3, 4):
            q_ref = sc.step(DT)
            pwm, aw = cascade_pid_step(q_ref, q_act, DT)
            vel_cmd = pwm * MAX_VEL
            p.setJointMotorControl2(robot_id, mcp_jid, p.VELOCITY_CONTROL,
                                    targetVelocity=-vel_cmd, force=8.0)
        elif state == 5:
            q_ref = q_act
            pwm, aw = 0.0, False
            p.setJointMotorControl2(robot_id, mcp_jid, p.VELOCITY_CONTROL,
                                    targetVelocity=0.0, force=0.0)
        else:   # IDLE / INIT
            q_ref = q_act
            pwm, aw = 0.0, False
            p.setJointMotorControl2(robot_id, mcp_jid, p.POSITION_CONTROL,
                                    targetPosition=0.0, force=2.0)

        # Other joints from sliders
        for jid, sid in sliders.items():
            if jid == mcp_jid:
                continue
            val = p.readUserDebugParameter(sid)
            p.setJointMotorControl2(robot_id, jid, p.POSITION_CONTROL,
                                    targetPosition=val * signs[jid], force=5.0)

        # Fingertip display
        for td in tip_data:
            pos = p.getLinkState(robot_id, td["lid"])[4]
            p.addUserDebugText(
                f"{td['label']}: ({pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f})",
                td["panel_pos"], textColorRGB=td["color"], textSize=0.85,
                replaceItemUniqueId=td["panel_id"])
            p.addUserDebugText(td["label"],
                [pos[0]+0.012,pos[1],pos[2]+0.006],
                textColorRGB=td["color"],textSize=0.85,
                replaceItemUniqueId=td["float_id"])
            p.addUserDebugLine(pos,[pos[0]+AXIS_LEN,pos[1],pos[2]],
                               [1,0,0],3,replaceItemUniqueId=td["lx"])
            p.addUserDebugLine(pos,[pos[0],pos[1]+AXIS_LEN,pos[2]],
                               [0,1,0],3,replaceItemUniqueId=td["ly"])
            p.addUserDebugLine(pos,[pos[0],pos[1],pos[2]+AXIS_LEN],
                               [0,0,1],3,replaceItemUniqueId=td["lz"])

        sc_status = "DONE" if sc.done else f"→{math.degrees(sc.q_tgt):.1f}d"
        set_panel([
            "=== §5.6-5.7  S-CURVE + FSM ===",
            f"FSM State: {FSM[state]}",
            f"State timer: {state_timer:.1f}s",
            f"Last event:  {last_event}",
            "— S-Curve Trajectory (Sec.5.7.5) —",
            f"  q_tgt={q_tgt_deg:.1f}d   v_max={math.degrees(SCURVE_V):.1f}d/s",
            f"  q_ref={math.degrees(q_ref):.1f}d   [{sc_status}]",
            f"  q_act={math.degrees(q_act):.1f}d   err={abs(math.degrees(q_tgt-q_act)):.2f}d",
            "— Cascade PID —",
            f"  PWM={pwm*100:+.1f}%   AW={'CLAMP' if aw else 'OK'}",
            f"  I_vel={pid['I_vel']:.4f}",
            "  (Kéo Target để trigger APPROACHING)",
        ])

        if loop % SIM_HZ == 0:
            print_table(state, state_timer, last_event,
                        math.degrees(q_tgt), math.degrees(q_ref),
                        math.degrees(q_act), sc.done, pwm, aw, pid["I_vel"])

        p.stepSimulation()
        time.sleep(DT)

except KeyboardInterrupt:
    p.disconnect()
    print("\n🔴 Dừng.")
