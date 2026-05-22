"""
Ch.5 §5.3.8 — Kinematic Singularity Analysis
Giao diện: fk_test.py style

Validate:
  - det(J) live cho Index finger           (Sec.5.3.8)
  - Singular configuration: q1=q2=0        (full extension)
  - FK reach vs max reach                  (Eq.5.16-5.17)
  - Màu cảnh báo khi gần singular:
      xanh lá → OK  |  vàng → near  |  đỏ → SINGULAR

Panel phải:
  - 5 fingertip positions (live)
  - Jacobian J, det(J), reach, trạng thái
  - Kiểm tra numerical tại q1=q2=0 (thesis: det=0)

Cách dùng:
  - Kéo Index MCP và PIP về 0 → det(J) tiến về 0
  - Kéo ngón uốn hết → det(J) lớn, nonsingular
"""
import pybullet as p
import pybullet_data
import time, os, math

exec(open(os.path.expanduser("~/bionic_hand_sim/fix_urdf.py")).read())
URDF = os.path.expanduser("~/bionic_hand_sim/assembly_1/urdf/assembly_1_fixed.urdf")

# ── Thesis constants ───────────────────────────────────────────────────────
L1, L2, L3 = 39.2, 30.0, 31.0
C           = 0.67
MAX_REACH   = L1 + L2 + L3       # 100.2 mm — full extension = singular

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
for i in range(11):
    tid = p.addUserDebugText("", [0.20, 0.0, 0.20 - i*0.043],
                             textColorRGB=[1,1,1], textSize=0.78)
    ALGO_IDS.append(tid)

# Warning label (changes color based on singularity)
warn_id = p.addUserDebugText("", [0.20, 0.0, 0.25],
                             textColorRGB=[0,1,0], textSize=1.1)

def set_panel(lines, warn_txt, warn_color):
    for i, tid in enumerate(ALGO_IDS):
        txt = lines[i] if i < len(lines) else ""
        p.addUserDebugText(txt, [0.20, 0.0, 0.20 - i*0.043],
                           textColorRGB=[1,1,1], textSize=0.78,
                           replaceItemUniqueId=tid)
    p.addUserDebugText(warn_txt, [0.20, 0.0, 0.25],
                       textColorRGB=warn_color, textSize=1.1,
                       replaceItemUniqueId=warn_id)

# ── Kinematics ─────────────────────────────────────────────────────────────
def fk_local(q1, q2):
    q3 = C * q2
    x = L1*math.sin(q1) + L2*math.sin(q1+q2) + L3*math.sin(q1+q2+q3)
    z = L1*math.cos(q1) + L2*math.cos(q1+q2) + L3*math.cos(q1+q2+q3)
    return x, z

def jacobian(q1, q2):
    q12  = q1 + q2
    q123 = q1 + q2 + C*q2
    J11 =  L1*math.cos(q1)  + L2*math.cos(q12)        + L3*math.cos(q123)
    J12 =  L2*math.cos(q12) + L3*(1+C)*math.cos(q123)
    J21 = -L1*math.sin(q1)  - L2*math.sin(q12)        - L3*math.sin(q123)
    J22 = -L2*math.sin(q12) - L3*(1+C)*math.sin(q123)
    return J11, J12, J21, J22

def det_j(q1, q2):
    J11, J12, J21, J22 = jacobian(q1, q2)
    return J11*J22 - J12*J21

# Numerical verification: singular at q1=q2=0
d_ext = det_j(0, 0)

mcp_jid = name_to_id["index_mcp_0"]
pip_jid = name_to_id["index_pip_0"]

def print_table(q1, q2, J11, J12, J21, J22, d, x_tip, z_tip, reach, status):
    bar_len  = 30
    norm_d   = min(abs(d) / 5000.0, 1.0)
    filled   = int(norm_d * bar_len)
    bar      = "█" * filled + "░" * (bar_len - filled)
    rows = [
        "┌──────────────────────────────────────────────────────────┐",
        "│       Ch.5 §5.3.8  SINGULARITY ANALYSIS                 │",
        "├──────────────────────┬───────────────────────────────────┤",
        "│  Joints (Index)      │  Jacobian J (Eq.5.27)            │",
        "├──────────────────────┼───────────────────────────────────┤",
        f"│  q1 (MCP) = {math.degrees(q1):>6.2f}°  │  J11={J11:>+8.2f}  J12={J12:>+8.2f}  │",
        f"│  q2 (PIP) = {math.degrees(q2):>6.2f}°  │  J21={J21:>+8.2f}  J22={J22:>+8.2f}  │",
        "├──────────────────────┴───────────────────────────────────┤",
        f"│  det(J) = {d:>+10.3f} mm²                                │",
        f"│  [{bar}] {norm_d*100:.0f}%          │",
        f"│  Status : {status:<47} │",
        "├──────────────────────────────────────────────────────────┤",
        f"│  FK  X={x_tip:>+7.2f}mm  Z={z_tip:>+7.2f}mm                      │",
        f"│  Reach = {reach:.2f} / {MAX_REACH:.1f} mm  ({reach/MAX_REACH*100:.0f}% extended)          │",
        f"│  Thesis check @ q1=q2=0: det={d_ext:.6f}  ✓            │",
        "└──────────────────────────────────────────────────────────┘",
    ]
    print(f"\033[{len(rows)}A", end="")
    for r in rows:
        print(f"\r{r}\033[K")

print("\n🟢 Ch.5 §5.3.8 — Singularity Analysis")
print(f"   Numerical check q1=q2=0: det(J)={d_ext:.6f}  (thesis: 0.0000)")
print("   Kéo Index MCP+PIP về 0 → đỏ SINGULAR. Uốn ngón → xanh OK.\n")
for _ in range(16):
    print()

loop = 0
try:
    while True:
        loop += 1

        q1 = p.readUserDebugParameter(sliders[mcp_jid])
        q2 = p.readUserDebugParameter(sliders[pip_jid])

        # Drive all joints
        for jid, sid in sliders.items():
            val = p.readUserDebugParameter(sid)
            p.setJointMotorControl2(robot_id, jid, p.POSITION_CONTROL,
                                    targetPosition=val * signs[jid], force=5.0)

        # Compute Jacobian & det
        J11, J12, J21, J22 = jacobian(q1, q2)
        d = J11*J22 - J12*J21

        # FK
        x_tip, z_tip = fk_local(q1, q2)
        reach = math.sqrt(x_tip**2 + z_tip**2)

        # Singularity status
        if abs(d) < 50:
            status = "!!! SINGULAR !!!"
            warn_color = [1.0, 0.1, 0.1]
            warn_txt   = f"!!! SINGULAR  det={d:.1f} !!!"
        elif abs(d) < 500:
            status = "  NEAR SINGULAR"
            warn_color = [1.0, 1.0, 0.0]
            warn_txt   = f"NEAR SINGULAR  det={d:.1f}"
        else:
            status = "  OK — nonsingular"
            warn_color = [0.2, 1.0, 0.2]
            warn_txt   = f"Nonsingular  det={d:.1f}"

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

        set_panel([
            "=== §5.3.8 SINGULARITY ANALYSIS ===",
            f"q1(MCP)={math.degrees(q1):.1f}d   q2(PIP)={math.degrees(q2):.1f}d",
            "— Jacobian J (Eq.5.27) —",
            f"  J11={J11:+.2f}  J12={J12:+.2f}",
            f"  J21={J21:+.2f}  J22={J22:+.2f}",
            "— det(J) = J11·J22 − J12·J21 —",
            f"  det(J) = {d:+.2f} mm²",
            f"  {status}",
            f"— FK reach = {reach:.1f} / {MAX_REACH:.1f} mm ({reach/MAX_REACH*100:.0f}%) —",
            f"  X={x_tip:+.2f}mm   Z={z_tip:+.2f}mm",
            f"  Singular @ q1=q2=0: det={d_ext:.4f} (thesis: 0.0000) ✓",
        ], warn_txt, warn_color)

        if loop % 240 == 0:
            print_table(q1, q2, J11, J12, J21, J22, d, x_tip, z_tip, reach, status.strip())

        p.stepSimulation()
        time.sleep(1.0 / 240.0)

except KeyboardInterrupt:
    p.disconnect()
    print("\n🔴 Dừng.")
