"""
Forward kinematics viewer.
- Same sliders as load_test.py
- Each fingertip: floating label follows the tip + XYZ axis marker (R=X, G=Y, B=Z)
- Panel on the right: live (x, y, z) in global frame for all 5 tips
"""
import pybullet as p
import pybullet_data
import time, os

exec(open(os.path.expanduser("~/bionic_hand_sim/fix_urdf.py")).read())

URDF = os.path.expanduser("~/bionic_hand_sim/assembly_1/urdf/assembly_1_fixed.urdf")

negative_axis_joints = [
    "index_mcp_0", "little_mcp_0", "ring_mcp_0",
    "index_pip_0",  "middle_pip_0", "ring_dip_0",
    "thumb_ip_0",
]
custom_ranges = {
    "thumb_dmc_0": (-0.5236, 2.0944, 0.0),
    "thumb_mcp_0": (-0.5236, 2.0944, 0.0),
}

# Fingertip links to track
TIPS = [
    ("thumb_dp_2_1", "THUMB",  [1.0, 0.55, 0.0]),
    ("dp_10_3_1",    "INDEX",  [0.2, 1.0,  0.2]),
    ("dp_10_3_3",    "MIDDLE", [0.2, 0.6,  1.0]),
    ("dp_10_3_2",    "RING",   [1.0, 0.2,  1.0]),
    ("dp_10_3",      "LITTLE", [1.0, 1.0,  0.2]),
]

AXIS_LEN = 0.010  # length of XYZ axis lines drawn at each tip (metres)

# ── Connect ───────────────────────────────────────────────────
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.resetDebugVisualizerCamera(
    cameraDistance=0.5, cameraYaw=45,
    cameraPitch=-45, cameraTargetPosition=[0, 0, 0.1]
)

p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"))
robot_id = p.loadURDF(
    URDF,
    basePosition=[0, 0, 0.15],
    baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
    useFixedBase=True
)

# ── Build maps ─────────────────────────────────────────────────
name_to_id = {}
link_id    = {}
for i in range(p.getNumJoints(robot_id)):
    info = p.getJointInfo(robot_id, i)
    name_to_id[info[1].decode()]  = i
    link_id[info[12].decode()]    = i

# ── Sliders (identical to load_test.py) ───────────────────────
finger_groups = {
    "Thumb":  ["thumb_dmc_0", "thumb_mcp_0", "thumb_ip_0"],
    "Index":  ["index_mcp_0",  "index_pip_0",  "index_dip_0"],
    "Middle": ["middle_mcp_0", "middle_pip_0", "middle_dip_0"],
    "Ring":   ["ring_mcp_0",   "ring_pip_0",   "ring_dip_0"],
    "Little": ["little_mcp_0", "little_pip_0", "little_dip_0"],
}

sliders = {}
signs   = {}
for finger, joints in finger_groups.items():
    for jname in joints:
        if jname not in name_to_id:
            continue
        jid = name_to_id[jname]
        label = f"{finger} {jname.split('_')[1].upper()}"
        lo, hi, default = custom_ranges.get(jname, (0.0, 1.5708, 0.0))
        sliders[jid] = p.addUserDebugParameter(label, lo, hi, default)
        signs[jid]   = -1 if jname in negative_axis_joints else 1

# ── FK display init ────────────────────────────────────────────
tip_data = []

for idx, (link_name, label, color) in enumerate(TIPS):
    lid = link_id.get(link_name)
    if lid is None:
        print(f"  ⚠  link '{link_name}' not found")
        continue

    pos = p.getLinkState(robot_id, lid)[4]   # worldLinkFramePosition

    # Floating label — follows the fingertip in 3D
    float_id = p.addUserDebugText(
        label,
        [pos[0] + 0.012, pos[1], pos[2] + 0.006],
        textColorRGB=color, textSize=0.85
    )

    # Panel text — fixed world position, right of the hand
    panel_pos = [0.20, 0.0, 0.46 - idx * 0.055]
    panel_id  = p.addUserDebugText(
        f"{label}: ({pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f})",
        panel_pos, textColorRGB=color, textSize=0.85
    )

    # 3D axis cross at fingertip: R=X  G=Y  B=Z
    lx = p.addUserDebugLine(pos, [pos[0]+AXIS_LEN, pos[1],          pos[2]         ], [1,0,0], lineWidth=3, lifeTime=0)
    ly = p.addUserDebugLine(pos, [pos[0],          pos[1]+AXIS_LEN, pos[2]         ], [0,1,0], lineWidth=3, lifeTime=0)
    lz = p.addUserDebugLine(pos, [pos[0],          pos[1],          pos[2]+AXIS_LEN], [0,0,1], lineWidth=3, lifeTime=0)

    tip_data.append({
        "lid": lid, "label": label, "color": color,
        "float_id": float_id,
        "panel_id": panel_id, "panel_pos": panel_pos,
        "lx": lx, "ly": ly, "lz": lz,
    })

print("\n🟢 FK Viewer  |  trục tại đầu ngón: X=đỏ  Y=xanh lá  Z=xanh dương")
print(f"   Tracking {len(tip_data)}/{len(TIPS)} tips: {[td['label'] for td in tip_data]}")
print("   Kéo slider → xem tọa độ update real-time.  Ctrl+C dừng.\n")
print(f"  {'FINGER':<8}  {'X':>8}  {'Y':>8}  {'Z':>8}")
print(f"  {'-'*38}")

# ── Main loop ──────────────────────────────────────────────────
loop_count = 0
try:
    while True:
        loop_count += 1
        # Drive joints from sliders
        for jid, slider_id in sliders.items():
            val = p.readUserDebugParameter(slider_id)
            p.setJointMotorControl2(
                bodyUniqueId=robot_id, jointIndex=jid,
                controlMode=p.POSITION_CONTROL,
                targetPosition=val * signs[jid],
                force=5.0
            )

        # Update all FK readouts
        for td in tip_data:
            pos = p.getLinkState(robot_id, td["lid"])[4]  # global frame

            # Floating label follows tip
            p.addUserDebugText(
                td["label"],
                [pos[0] + 0.012, pos[1], pos[2] + 0.006],
                textColorRGB=td["color"], textSize=0.85,
                replaceItemUniqueId=td["float_id"]
            )

            # Panel readout
            p.addUserDebugText(
                f"{td['label']}: ({pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f})",
                td["panel_pos"],
                textColorRGB=td["color"], textSize=0.85,
                replaceItemUniqueId=td["panel_id"]
            )

            # Axis cross at tip
            td["lx"] = p.addUserDebugLine(
                pos, [pos[0]+AXIS_LEN, pos[1],          pos[2]         ],
                [1,0,0], lineWidth=3, lifeTime=0, replaceItemUniqueId=td["lx"]
            )
            td["ly"] = p.addUserDebugLine(
                pos, [pos[0],          pos[1]+AXIS_LEN, pos[2]         ],
                [0,1,0], lineWidth=3, lifeTime=0, replaceItemUniqueId=td["ly"]
            )
            td["lz"] = p.addUserDebugLine(
                pos, [pos[0],          pos[1],          pos[2]+AXIS_LEN],
                [0,0,1], lineWidth=3, lifeTime=0, replaceItemUniqueId=td["lz"]
            )

        # Print all positions to console every 1 s
        if loop_count % 240 == 0:
            print(f"\033[{len(tip_data)+1}A", end="")  # move cursor up to overwrite
            for td in tip_data:
                pos = p.getLinkState(robot_id, td["lid"])[4]
                print(f"  {td['label']:<8}  {pos[0]:>+8.4f}  {pos[1]:>+8.4f}  {pos[2]:>+8.4f}")

        p.stepSimulation()
        time.sleep(1. / 240.)

except KeyboardInterrupt:
    p.disconnect()
    print("🔴 Dừng.")
