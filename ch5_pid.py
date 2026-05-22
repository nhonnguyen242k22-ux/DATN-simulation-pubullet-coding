"""
Ch.5 §5.5 — Cascade Position-Velocity PID + Anti-Windup
Giao diện: fk_test.py style

Validate:
  - Outer P loop:  vel_ref = Kp_pos · e_pos          (Eq.5.29)
  - Inner PI loop: PWM = Kp_vel·e_vel + Ki_vel·∫e_vel (Eq.5.31–5.34)
  - EMA velocity filter                               (Eq.5.35)
  - Anti-Windup clamping @ PWM ≥ 90%                 (Sec.5.5.4)
  - Hold mode 30% sau T_hold giây saturation

Điều khiển: 1 ngón tay (Index MCP).
  - Slider "PID Target": đặt góc mục tiêu (0–90°)
  - Cascade PID chạy trong vòng lặp sim → drive qua VELOCITY_CONTROL
  - Panel phải hiện live: e_pos, vel_ref, e_vel, I_vel, PWM, trạng thái AW

Slider "Sim Joint Force": cho phép chặn ngón (tăng lực cản) → test Anti-Windup.
"""
import pybullet as p
import pybullet_data
import time, os, math

exec(open(os.path.expanduser("~/bionic_hand_sim/fix_urdf.py")).read())
URDF = os.path.expanduser("~/bionic_hand_sim/assembly_1/urdf/assembly_1_fixed.urdf")

# ── Controller parameters (§5.5) ──────────────────────────────────────────
KP_POS     = 8.0        # outer P gain (s⁻¹)             Eq.5.30
KP_VEL     = 0.10       # inner PI proportional           (sim-scaled)
KI_VEL     = 1.5        # inner PI integral               (sim-scaled)
MAX_VEL    = math.pi    # velocity saturation (rad/s)
PWM_MAX    = 0.90       # anti-windup threshold           Sec.5.5.4
T_HOLD_MAX = 5.0        # hold mode timeout (s)
DEADBAND   = math.radians(0.5)   # dead-band on position error
EMA_TAU    = 0.005      # EMA filter τ (s)               Eq.5.35

SIM_HZ = 240
DT     = 1.0 / SIM_HZ
EMA_A  = DT / (DT + EMA_TAU)

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

# ── Maps ───────────────────────────────────────────────────────────────────
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
            f"{finger} {jname.split('_')[1].upper()}", lo, hi, default
        )
        signs[jid] = -1 if jname in negative_axis_joints else 1

# ── PID-specific controls ──────────────────────────────────────────────────
pid_target_sl = p.addUserDebugParameter("PID Target (deg)",   0.0, 88.0, 40.0)
joint_force_sl= p.addUserDebugParameter("Joint Resist Force", 0.5, 20.0,  5.0)

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
    pid_txt = p.addUserDebugText(
        f"{label}: ({pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f})",
        pp, textColorRGB=color, textSize=0.85)
    fid = p.addUserDebugText(label,
        [pos[0]+0.012,pos[1],pos[2]+0.006], textColorRGB=color, textSize=0.85)
    lx = p.addUserDebugLine(pos,[pos[0]+AXIS_LEN,pos[1],pos[2]],[1,0,0],3)
    ly = p.addUserDebugLine(pos,[pos[0],pos[1]+AXIS_LEN,pos[2]],[0,1,0],3)
    lz = p.addUserDebugLine(pos,[pos[0],pos[1],pos[2]+AXIS_LEN],[0,0,1],3)
    tip_data.append({"lid":lid,"label":label,"color":color,
                     "panel_id":pid_txt,"panel_pos":pp,"float_id":fid,
                     "lx":lx,"ly":ly,"lz":lz})

# ── Algorithm panel ────────────────────────────────────────────────────────
ALGO_IDS = []
for i in range(11):
    tid = p.addUserDebugText("", [0.20, 0.0, 0.20 - i*0.043],
                             textColorRGB=[1,1,1], textSize=0.78)
    ALGO_IDS.append(tid)

def set_panel(lines):
    for i, tid in enumerate(ALGO_IDS):
        txt = lines[i] if i < len(lines) else ""
        p.addUserDebugText(txt, [0.20, 0.0, 0.20 - i*0.043],
                           textColorRGB=[1,1,1], textSize=0.78,
                           replaceItemUniqueId=tid)

# ── PID state ─────────────────────────────────────────────────────────────
mcp_jid   = name_to_id["index_mcp_0"]
pid_state = {"I_vel": 0.0, "vel_filt": 0.0, "prev_q": 0.0, "sat_timer": 0.0}

def cascade_pid(q_ref, q_act, dt, force):
    st = pid_state
    # Outer P (Eq.5.29)
    e_pos = q_ref - q_act
    if abs(e_pos) < DEADBAND:
        e_pos = 0.0
    vel_ref = max(-MAX_VEL, min(MAX_VEL, KP_POS * e_pos))

    # EMA velocity (Eq.5.35)
    vel_raw = (q_act - st["prev_q"]) / dt
    st["vel_filt"] = EMA_A * vel_raw + (1.0 - EMA_A) * st["vel_filt"]
    st["prev_q"] = q_act

    # Inner PI (Eq.5.31–5.34)
    e_vel   = vel_ref - st["vel_filt"]
    pwm_raw = KP_VEL * e_vel + st["I_vel"]

    # Anti-Windup (Sec.5.5.4)
    if abs(pwm_raw) >= PWM_MAX:
        st["sat_timer"] += dt
        aw = True
    else:
        st["I_vel"] += KI_VEL * e_vel * dt
        st["sat_timer"] = 0.0
        aw = False

    pwm = max(-1.0, min(1.0, pwm_raw))
    hold = st["sat_timer"] > T_HOLD_MAX
    if hold:
        pwm = math.copysign(0.30, pwm_raw)

    # Apply via VELOCITY_CONTROL (negative_axis flip)
    vel_cmd = pwm * MAX_VEL
    p.setJointMotorControl2(robot_id, mcp_jid, p.VELOCITY_CONTROL,
                            targetVelocity=-vel_cmd, force=force)
    return e_pos, vel_ref, st["vel_filt"], e_vel, st["I_vel"], pwm, aw, hold

def print_table(q_tgt_d, q_act_d, force, ep, vr, va, ev, iv, pwm, aw, hold, sat_t):
    aw_s  = "CLAMP ⚠" if aw   else "OK    "
    hd_s  = "HOLD-30%" if hold else "        "
    rows = [
        "┌──────────────────────────────────────────────────────────┐",
        "│       Ch.5 §5.5  CASCADE PID + ANTI-WINDUP              │",
        f"│  Kp_pos={KP_POS}   Kp_vel={KP_VEL}   Ki_vel={KI_VEL}   PWM_max={PWM_MAX*100:.0f}%      │",
        "├─────────────────────────────┬────────────────────────────┤",
        "│  Outer P Loop (Eq.5.29)     │  Inner PI Loop (Eq.5.31)  │",
        "├─────────────────────────────┼────────────────────────────┤",
        f"│  q_target  = {q_tgt_d:>7.2f} deg    │  vel_act = {va:>+8.2f} d/s    │",
        f"│  q_actual  = {q_act_d:>7.2f} deg    │  e_vel   = {ev:>+8.2f} d/s    │",
        f"│  e_pos     = {ep:>+7.2f} deg    │  I_vel   = {iv:>+8.4f}        │",
        f"│  vel_ref   = {vr:>+7.2f} d/s    │  PWM     = {pwm*100:>+8.2f} %      │",
        "├─────────────────────────────┴────────────────────────────┤",
        f"│  Anti-Windup: [{aw_s}]   {hd_s}   sat={sat_t:.1f}s/{T_HOLD_MAX}s  │",
        f"│  Joint resist force = {force:.1f} N  (kéo cao → block → test AW)    │",
        "└──────────────────────────────────────────────────────────┘",
    ]
    print(f"\033[{len(rows)}A", end="")
    for r in rows:
        print(f"\r{r}\033[K")

print("\n🟢 Ch.5 §5.5 — Cascade PID + Anti-Windup")
print(f"   Kp_pos={KP_POS}  Kp_vel={KP_VEL}  Ki_vel={KI_VEL}  PWM_max={PWM_MAX*100:.0f}%")
print("   Kéo 'PID Target' để đặt góc mục tiêu. Kéo 'Joint Force' lên để block → AW.\n")
for _ in range(14):
    print()

loop = 0
prev_tgt = -999.0

try:
    while True:
        loop += 1

        q_tgt_deg = p.readUserDebugParameter(pid_target_sl)
        q_tgt     = math.radians(q_tgt_deg)
        force     = p.readUserDebugParameter(joint_force_sl)

        # q_actual: index_mcp is negative_axis → flip sign
        q_act = max(0.0, -p.getJointState(robot_id, mcp_jid)[0])

        # Reset integrator on large target jump
        if abs(q_tgt - prev_tgt) > math.radians(5):
            pid_state["I_vel"] = 0.0
            pid_state["sat_timer"] = 0.0
            prev_tgt = q_tgt

        e_pos, vel_ref, vel_act, e_vel, I_vel, pwm, aw, hold = \
            cascade_pid(q_tgt, q_act, DT, force)

        # Drive all OTHER joints from sliders normally
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

        aw_str   = "AW-CLAMP" if aw else "OK"
        hold_str = "  ← HOLD-30%" if hold else ""

        set_panel([
            "=== §5.5 CASCADE PID + ANTI-WINDUP ===",
            f"q_tgt={q_tgt_deg:.1f}d   q_act={math.degrees(q_act):.1f}d   Force={force:.1f}N",
            f"Kp_pos={KP_POS}  Kp_vel={KP_VEL}  Ki_vel={KI_VEL}",
            "— Outer P Loop (Eq.5.29) —",
            f"  e_pos = {math.degrees(e_pos):+.2f} deg",
            f"  vel_ref = Kp·e = {math.degrees(vel_ref):+.1f} deg/s",
            "— Inner PI Loop (Eq.5.31-34) —",
            f"  vel_act={math.degrees(vel_act):+.1f}d/s   e_vel={math.degrees(e_vel):+.1f}d/s",
            f"  I_vel={I_vel:.4f}   PWM={pwm*100:+.1f}%",
            f"  Anti-Windup: [{aw_str}]{hold_str}",
            f"  sat_timer={pid_state['sat_timer']:.2f}s / {T_HOLD_MAX}s",
        ])

        if loop % SIM_HZ == 0:
            print_table(q_tgt_deg, math.degrees(q_act), force,
                        math.degrees(e_pos), math.degrees(vel_ref),
                        math.degrees(vel_act), math.degrees(e_vel),
                        I_vel, pwm, aw, hold, pid_state["sat_timer"])

        p.stepSimulation()
        time.sleep(DT)

except KeyboardInterrupt:
    p.disconnect()
    print("\n🔴 Dừng.")
