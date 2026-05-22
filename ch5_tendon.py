"""
Ch.5 §5.3 — Tendon Routing Matrix + Coupling + Static Decoupler
Giao diện: fk_test.py style

Validate:
  - h = R_tendon · q           (Eq.5.1–5.2)  cable length change
  - τ = R_tendon^T · T         (Eq.5.3–5.4)  torque distribution
  - q3 = C · q2                (Eq.5.7)       underactuated coupling
  - θ_m1 = (r11/rm1)·u1       (Eq.5.8)       motor 1 ref
  - θ_m2 = (r21/rm2)·u1 + ((r22+C·r23)/rm2)·u2  (Eq.5.10)  motor 2 ref

Panel phải:
  - 5 fingertip positions (live, same as fk_test)
  - Tendon / decoupler readout cho index finger

Dip slider của Index bị khoá = C · Pip → mô phỏng đúng cơ chế underactuated.
"""
import pybullet as p
import pybullet_data
import time, os, math

exec(open(os.path.expanduser("~/bionic_hand_sim/fix_urdf.py")).read())
URDF = os.path.expanduser("~/bionic_hand_sim/assembly_1/urdf/assembly_1_fixed.urdf")

# ── Thesis §5.3 constants ──────────────────────────────────────────────────
C            = 0.67          # coupling ratio q3 = C·q2 (Eq.5.7)
R11, R21     = 6.0, 4.0     # mm  — pulley radii at MCP  (Cable 1 / Cable 2)
R22, R23     = 6.0, 6.0     # mm  — pulley radii at PIP, DIP (Cable 2)
RM1, RM2     = 6.0, 6.0     # mm  — motor spool radii

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

# ── Joint / link maps ──────────────────────────────────────────────────────
name_to_id = {}
link_id    = {}
for i in range(p.getNumJoints(robot_id)):
    info = p.getJointInfo(robot_id, i)
    name_to_id[info[1].decode()]  = i
    link_id[info[12].decode()]    = i

# ── Config (same as fk_test.py) ────────────────────────────────────────────
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

# ── Sliders (all fingers) ──────────────────────────────────────────────────
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
        pp, textColorRGB=color, textSize=0.85
    )
    fid = p.addUserDebugText(label,
        [pos[0]+0.012, pos[1], pos[2]+0.006], textColorRGB=color, textSize=0.85)
    lx = p.addUserDebugLine(pos, [pos[0]+AXIS_LEN,pos[1],pos[2]], [1,0,0], 3)
    ly = p.addUserDebugLine(pos, [pos[0],pos[1]+AXIS_LEN,pos[2]], [0,1,0], 3)
    lz = p.addUserDebugLine(pos, [pos[0],pos[1],pos[2]+AXIS_LEN], [0,0,1], 3)
    tip_data.append({"lid":lid,"label":label,"color":color,
                     "panel_id":pid,"panel_pos":pp,"float_id":fid,
                     "lx":lx,"ly":ly,"lz":lz})

# ── Algorithm output panel (9 rows, below tip panel) ──────────────────────
ALGO_IDS = []
for i in range(10):
    tid = p.addUserDebugText(
        "", [0.20, 0.0, 0.20 - i*0.045],
        textColorRGB=[1,1,1], textSize=0.78
    )
    ALGO_IDS.append(tid)

def set_panel(lines):
    for i, tid in enumerate(ALGO_IDS):
        txt = lines[i] if i < len(lines) else ""
        p.addUserDebugText(txt, [0.20, 0.0, 0.20 - i*0.045],
                           textColorRGB=[1,1,1], textSize=0.78,
                           replaceItemUniqueId=tid)

# ── Index finger joint IDs ─────────────────────────────────────────────────
mcp_jid = name_to_id["index_mcp_0"]
pip_jid = name_to_id["index_pip_0"]
dip_jid = name_to_id["index_dip_0"]

NROWS = 12   # số dòng bảng terminal (để cursor-up đúng)

def print_table(q1, q2, q3, h1, h2, h2_check, tau1, tau2, tau3, tm1, tm2, dc):
    d = math.degrees
    rows = [
        "┌─────────────────────────────────────────────────────────┐",
        "│       Ch.5 §5.3  TENDON ROUTING + DECOUPLER            │",
        "├──────────────┬──────────────┬───────────────────────────┤",
        "│   Joint      │  Angle (deg) │  Notes                    │",
        "├──────────────┼──────────────┼───────────────────────────┤",
        f"│  q1  MCP     │  {d(q1):>8.2f}°  │  slider                   │",
        f"│  q2  PIP     │  {d(q2):>8.2f}°  │  slider                   │",
        f"│  q3  DIP     │  {d(q3):>8.2f}°  │  = C·q2  (coupling)       │",
        "├──────────────┴──────────────┴───────────────────────────┤",
        "│  Routing h = R·q (Eq.5.2)                              │",
        f"│  h1 = r11·q1              = {h1:>+8.4f} mm               │",
        f"│  h2 = r21q1+r22q2+r23q3  = {h2:>+8.4f} mm  (✓={h2_check:.4f})│",
        "│  Torque τ = R^T·T  @ T=[1,1]N (Eq.5.4)                │",
        f"│  τ_MCP={tau1:.1f}  τ_PIP={tau2:.1f}  τ_DIP={tau3:.1f}  N·mm                     │",
        "│  Static Decoupler (Eq.5.8-5.10)                        │",
        f"│  θm1 = (r11/rm1)·q1        = {d(tm1):>8.3f}°              │",
        f"│  θm2 = k_cross·q1 + k·q2  = {d(tm2):>8.3f}°  (comp={d(dc):.2f}°)│",
        "└─────────────────────────────────────────────────────────┘",
    ]
    global NROWS
    NROWS = len(rows)
    print(f"\033[{NROWS}A", end="")   # move cursor up
    for r in rows:
        print(f"\r{r}\033[K")

print("\n🟢 Ch.5 §5.3 — Tendon Routing + Decoupler")
print(f"   R=[r11={R11} r21={R21} r22={R22} r23={R23}]  C={C}  rm1=rm2={RM1}")
print("   Kéo Index MCP / PIP → xem h1, h2, θm1, θm2. DIP tự khóa = C·PIP.\n")
# Pre-print blank rows so cursor-up works from the start
for _ in range(18):
    print()

loop = 0
try:
    while True:
        loop += 1

        # Read index MCP, PIP from sliders (always positive = flexion)
        q1 = p.readUserDebugParameter(sliders[mcp_jid])   # MCP
        q2 = p.readUserDebugParameter(sliders[pip_jid])   # PIP
        q3 = C * q2                                        # DIP (coupling Eq.5.7)

        # Drive all joints; DIP locked to coupling
        for jid, sid in sliders.items():
            val = p.readUserDebugParameter(sid)
            if jid == dip_jid:
                val = q3        # enforce coupling
            p.setJointMotorControl2(robot_id, jid, p.POSITION_CONTROL,
                                    targetPosition=val * signs[jid], force=5.0)

        # ── Tendon routing matrix h = R·q  (Eq.5.2) ───────────────────────
        h1 = R11 * q1
        h2 = R21 * q1 + R22 * q2 + R23 * q3     # q3 = C·q2 already

        # Equivalent: h2 = R21·q1 + (R22 + C·R23)·q2
        h2_check = R21*q1 + (R22 + C*R23)*q2    # must equal h2

        # ── Torque distribution τ = R^T·T  (Eq.5.4) ──────────────────────
        # Assume unit cable tensions T1=T2=1 N for illustration
        T1, T2 = 1.0, 1.0
        tau1 = R11*T1 + R21*T2    # MCP torque
        tau2 =          R22*T2    # PIP torque
        tau3 =          R23*T2    # DIP torque

        # ── Static Decoupler  (Eq.5.8–5.10) ──────────────────────────────
        tm1 = (R11 / RM1) * q1
        tm2 = (R21 / RM2) * q1 + ((R22 + C*R23) / RM2) * q2

        # Cross-coupling compensation term (the "decoupling" component)
        decouple_comp = (R21 / RM2) * q1   # amount Motor2 compensates for MCP motion

        # ── Update fingertip display ───────────────────────────────────────
        for td in tip_data:
            pos = p.getLinkState(robot_id, td["lid"])[4]
            p.addUserDebugText(
                f"{td['label']}: ({pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f})",
                td["panel_pos"], textColorRGB=td["color"], textSize=0.85,
                replaceItemUniqueId=td["panel_id"]
            )
            p.addUserDebugText(td["label"],
                [pos[0]+0.012, pos[1], pos[2]+0.006],
                textColorRGB=td["color"], textSize=0.85,
                replaceItemUniqueId=td["float_id"])
            p.addUserDebugLine(pos,[pos[0]+AXIS_LEN,pos[1],pos[2]],
                               [1,0,0],3,replaceItemUniqueId=td["lx"])
            p.addUserDebugLine(pos,[pos[0],pos[1]+AXIS_LEN,pos[2]],
                               [0,1,0],3,replaceItemUniqueId=td["ly"])
            p.addUserDebugLine(pos,[pos[0],pos[1],pos[2]+AXIS_LEN],
                               [0,0,1],3,replaceItemUniqueId=td["lz"])

        # ── Algorithm panel ────────────────────────────────────────────────
        set_panel([
            "=== §5.3 TENDON ROUTING + DECOUPLER ===",
            f"q1(MCP)={math.degrees(q1):5.1f}d  q2(PIP)={math.degrees(q2):5.1f}d  q3=C·q2={math.degrees(q3):5.1f}d",
            "— Routing Matrix h=R·q (Eq.5.2) —",
            f"  h1 = r11·q1         = {R11}·{q1:.3f} = {h1:+.3f} mm",
            f"  h2 = r21q1+r22q2+r23q3 = {h2:.3f} mm  (check={h2_check:.3f})",
            "— Torque R^T·T @ T=[1,1]N (Eq.5.4) —",
            f"  τ_MCP={tau1:.1f}  τ_PIP={tau2:.1f}  τ_DIP={tau3:.1f}  N·mm",
            "— Static Decoupler (Eq.5.8-5.10) —",
            f"  θm1 = (r11/rm1)·q1          = {math.degrees(tm1):.2f} deg",
            f"  θm2 = (r21/rm2)·q1+k·q2     = {math.degrees(tm2):.2f} deg  (comp={math.degrees(decouple_comp):.2f})",
        ])

        if loop % 240 == 0:
            print_table(q1, q2, q3, h1, h2, h2_check, tau1, tau2, tau3, tm1, tm2, decouple_comp)

        p.stepSimulation()
        time.sleep(1.0 / 240.0)

except KeyboardInterrupt:
    p.disconnect()
    print("\n🔴 Dừng.")
