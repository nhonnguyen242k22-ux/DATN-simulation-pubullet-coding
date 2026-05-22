"""
FK + IK viewer cho bionic hand.
- Panel cố định bên phải: hiện tọa độ (x, y, z) của 5 đầu ngón
- Slider đặt target (X, Y, Z), nút bấm chọn ngón → IK di chuyển
- Trục XYZ tại mỗi đầu ngón (R=X, G=Y, B=Z)
- Animation ease-in-out 2 giây
"""
import pybullet as p
import pybullet_data
import time, os, math

# ── Config ─────────────────────────────────────────────────────
SIM_HZ       = 240        # tỉ lệ: bước sim mỗi giây
JOINT_FORCE   = 200.0
POS_GAIN      = 8
ANIM_SEC      = 0.00001
AXIS_LEN      = 0.010     # trục XYZ tại đầu ngón (mét)
PANEL_X       = 0.25      # vị trí X cố định của panel text

# ── Controller selection ────────────────────────────────────────
# "position"   → PyBullet POSITION_CONTROL (PD, gains POS_GAIN/JOINT_FORCE)
# "pi_velocity" → PI trên vòng ngoài, output velocity → VELOCITY_CONTROL
CTRL_MODE = "pi_velocity"
KP        = 1.0    # tỉ lệ: (rad/s) / rad
KI        = 24.0    # tích phân: (rad/s) / (rad·s)
MAX_VEL   = 3.0    # giới hạn velocity command (rad/s)

exec(open(os.path.expanduser("~/bionic_hand_sim/fix_urdf.py")).read())
URDF = os.path.expanduser("~/bionic_hand_sim/assembly_1/urdf/assembly_1_fixed.urdf")

# ── Connect ────────────────────────────────────────────────────
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.resetDebugVisualizerCamera(
    cameraDistance=0.4, cameraYaw=45,
    cameraPitch=-45,    cameraTargetPosition=[0, 0, 0.1]
)

p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"))
robot_id = p.loadURDF(
    URDF,
    basePosition=[0, 0, 0.15],
    baseOrientation=p.getQuaternionFromEuler([0, 0, -math.pi / 2]),
    useFixedBase=True
)

# ── Build maps ─────────────────────────────────────────────────
joint_id = {}
link_id  = {}
movable_joints = []
ll, ul, jr, rp = [], [], [], []   # lower, upper, range, rest

for i in range(p.getNumJoints(robot_id)):
    info = p.getJointInfo(robot_id, i)
    jname = info[1].decode()
    lname = info[12].decode()
    joint_id[jname] = i
    link_id[lname]  = i

    if info[2] != p.JOINT_FIXED:
        movable_joints.append(jname)
        lo, hi = info[8], info[9]
        if lo == 0 and hi == -1:        # URDF không đặt limit
            lo, hi = -3.14, 3.14
        ll.append(lo); ul.append(hi)
        jr.append(hi - lo); rp.append(0.0)
        p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL,
                                targetPosition=0.0, force=JOINT_FORCE)

# ── Finger definitions ─────────────────────────────────────────
finger_groups = {
    "Thumb":  ["thumb_dmc_0", "thumb_mcp_0", "thumb_ip_0"],
    "Index":  ["index_mcp_0",  "index_pip_0",  "index_dip_0"],
    "Middle": ["middle_mcp_0", "middle_pip_0", "middle_dip_0"],
    "Ring":   ["ring_mcp_0",   "ring_pip_0",   "ring_dip_0"],
    "Little": ["little_mcp_0", "little_pip_0", "little_dip_0"],
}

TIPS = [
    ("thumb_dp_2_1", "Thumb",  [1.0, 0.55, 0.0]),
    ("dp_10_3_1",    "Index",  [0.2, 1.0,  0.2]),
    ("dp_10_3_3",    "Middle", [0.2, 0.6,  1.0]),
    ("dp_10_3_2",    "Ring",   [1.0, 0.2,  1.0]),
    ("dp_10_3",      "Little", [1.0, 1.0,  0.2]),
]

# ── UI: sliders + buttons ──────────────────────────────────────
slider_x = p.addUserDebugParameter("Target X (forward)",  0.0,  0.20, 0.10)
slider_y = p.addUserDebugParameter("Target Y (lateral)", -0.10, 0.10, 0.00)
slider_z = p.addUserDebugParameter("Target Z", 0.0, 0.25, 0.18)

finger_btns = {}
for fname in finger_groups:
    finger_btns[fname] = p.addUserDebugParameter(f"Move {fname}", 1, 0, 0)

# ── Panel cố định — hiện tọa độ 5 đầu ngón ────────────────────
p.addUserDebugText("=== FINGERTIP POSITIONS ===",
                   [PANEL_X, 0.0, 0.47], textColorRGB=[1, 1, 1], textSize=1.0)

tip_data = []
for idx, (lname, label, color) in enumerate(TIPS):
    lid = link_id.get(lname)
    if lid is None:
        print(f"  ⚠  link '{lname}' not found, bỏ qua")
        continue
    pos = p.getLinkState(robot_id, lid)[4]

    # Panel text cố định
    panel_pos = [PANEL_X, 0.0, 0.42 - idx * 0.05]
    panel_id  = p.addUserDebugText(
        f"    {label:<6}: ({pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f})",
        panel_pos, textColorRGB=color, textSize=0.9
    )

    # Trục XYZ tại đầu ngón
    lx = p.addUserDebugLine(pos, [pos[0]+AXIS_LEN, pos[1], pos[2]], [1,0,0], lineWidth=3)
    ly = p.addUserDebugLine(pos, [pos[0], pos[1]+AXIS_LEN, pos[2]], [0,1,0], lineWidth=3)
    lz = p.addUserDebugLine(pos, [pos[0], pos[1], pos[2]+AXIS_LEN], [0,0,1], lineWidth=3)

    tip_data.append({
        "lid": lid, "label": label, "color": color,
        "panel_id": panel_id, "panel_pos": panel_pos,
        "lx": lx, "ly": ly, "lz": lz,
    })

# Target marker
_O = [0.0, 0.0, 0.0]
target_visual_id = p.addUserDebugLine(_O, _O, [1, 0, 0], 5)

# ── Animation state ────────────────────────────────────────────
animating       = False
animation_steps = 0
current_step    = 0
start_poses     = {}
target_poses    = {}
selected_finger = "Index"

# global_target_poses: giữ góc hiện tại của tất cả movable joints
global_target_poses = {}
for jname in movable_joints:
    global_target_poses[joint_id[jname]] = 0.0

prev_btn_vals = {f: 0 for f in finger_groups}
pi_integral   = {jid: 0.0 for jid in global_target_poses}

# ── Smooth interpolation helper ────────────────────────────────
def ease_in_out(t):
    """Hermite ease-in-out: 3t² − 2t³"""
    return t * t * (3.0 - 2.0 * t)

# ── Home ───────────────────────────────────────────────────────
print("\n⏳ Đang về home...")
HOME_STEPS = max(1, int(0.1 * SIM_HZ))
for step in range(HOME_STEPS):
    t = ease_in_out(step / float(HOME_STEPS))
    for jid, cur in global_target_poses.items():
        ang = cur * (1.0 - t)
        p.setJointMotorControl2(robot_id, jid, p.POSITION_CONTROL,
                                targetPosition=ang, force=JOINT_FORCE,
                                positionGain=POS_GAIN)
    p.stepSimulation()
    time.sleep(1.0 / SIM_HZ)

for jid in global_target_poses:
    global_target_poses[jid] = 0.0
print("✅ Đã về home!\n")

# ── Print actual fingertip positions so user knows slider workspace ─────
print("── Vị trí đầu ngón khi về home (để căn chỉnh slider Target) ──")
for td in tip_data:
    pos = p.getLinkState(robot_id, td["lid"])[4]
    print(f"  {td['label']:<6}: x={pos[0]:+.4f}  y={pos[1]:+.4f}  z={pos[2]:+.4f}")
print()

# ── Main loop ──────────────────────────────────────────────────
print("🟢 Chạy!  Kéo slider → đặt target.  Bấm nút → IK di chuyển ngón.")
print(f"   Panel bên phải hiện tọa độ 5 đầu ngón.  Ctrl+C dừng.\n")

loop_count = 0
try:
    while True:
        loop_count += 1

        # ---- Đọc target từ slider ----
        target_pos = [
            p.readUserDebugParameter(slider_x),
            p.readUserDebugParameter(slider_y),
            p.readUserDebugParameter(slider_z),
        ]

        # ---- Kiểm tra nút bấm ----
        clicked_finger = None
        for fname, btn_id in finger_btns.items():
            val = p.readUserDebugParameter(btn_id)
            if val > prev_btn_vals[fname]:
                prev_btn_vals[fname] = val
                if not animating:
                    clicked_finger = fname

        # ---- Target marker ----
        p.addUserDebugLine(
            target_pos,
            [target_pos[0], target_pos[1], target_pos[2] + 0.02],
            [1, 0, 0], 5, replaceItemUniqueId=target_visual_id
        )

        # ---- Cập nhật panel + trục cho tất cả 5 ngón ----
        for td in tip_data:
            pos = p.getLinkState(robot_id, td["lid"])[4]

            # Highlight ngón đang chọn
            prefix = ">>>" if td["label"] == selected_finger else "   "
            p.addUserDebugText(
                f"{prefix} {td['label']:<6}: ({pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f})",
                td["panel_pos"], textColorRGB=td["color"], textSize=0.9,
                replaceItemUniqueId=td["panel_id"]
            )

            # Trục XYZ
            p.addUserDebugLine(pos, [pos[0]+AXIS_LEN, pos[1], pos[2]],
                               [1,0,0], lineWidth=3, replaceItemUniqueId=td["lx"])
            p.addUserDebugLine(pos, [pos[0], pos[1]+AXIS_LEN, pos[2]],
                               [0,1,0], lineWidth=3, replaceItemUniqueId=td["ly"])
            p.addUserDebugLine(pos, [pos[0], pos[1], pos[2]+AXIS_LEN],
                               [0,0,1], lineWidth=3, replaceItemUniqueId=td["lz"])

        # ---- Print terminal mỗi 1 giây ----
        if loop_count % SIM_HZ == 0:
            sec = loop_count // SIM_HZ
            for td in tip_data:
                pos = p.getLinkState(robot_id, td["lid"])[4]
                marker = "◀" if td["label"] == selected_finger else " "
                print(f"  {marker} {td['label']:<6}: x={pos[0]:+.4f}  y={pos[1]:+.4f}  z={pos[2]:+.4f}")
            print()

        # ---- IK khi bấm nút ----
        if clicked_finger and not animating:
            selected_finger = clicked_finger
            tip_link = None
            for lname, label, _ in TIPS:
                if label == selected_finger:
                    tip_link = lname
                    break
            tip_id = link_id[tip_link]

            # Use current joint angles as rest poses so IK finds nearby solutions
            rp_current = [p.getJointState(robot_id, joint_id[jn])[0] for jn in movable_joints]

            try:
                joint_poses = p.calculateInverseKinematics(
                    robot_id, tip_id, target_pos,
                    lowerLimits=ll, upperLimits=ul,
                    jointRanges=jr, restPoses=rp_current,
                    maxNumIterations=200,
                    residualThreshold=1e-4
                )
            except Exception as e:
                print(f"⚠ IK lỗi: {e}")
                joint_poses = None

            # Kiểm tra NaN trước khi dùng
            if joint_poses is None or any(abs(v) > 1e6 or v != v for v in joint_poses):
                print(f"⚠ IK không hội tụ cho {selected_finger} tại target {target_pos} — bỏ qua")
            else:
                ik_dict = {jname: joint_poses[i] for i, jname in enumerate(movable_joints)}

                # Clamp góc IK vào 90% giới hạn để tránh mesh tự giao nhau
                for i, jname in enumerate(movable_joints):
                    lo, hi = ll[i], ul[i]
                    margin = (hi - lo) * 0.05          # 5% margin mỗi đầu
                    ik_dict[jname] = max(lo + margin, min(hi - margin, ik_dict[jname]))

                start_poses.clear()
                target_poses.clear()
                for jname in finger_groups[selected_finger]:
                    jid = joint_id[jname]
                    start_poses[jid]  = p.getJointState(robot_id, jid)[0]
                    target_poses[jid] = ik_dict[jname]
                    global_target_poses[jid] = ik_dict[jname]

                for jname in finger_groups[selected_finger]:
                    pi_integral[joint_id[jname]] = 0.0

                animating       = True
                current_step    = 0
                animation_steps = max(1, int(ANIM_SEC * SIM_HZ))

                print(f"\n── IK: {selected_finger} → target {[f'{v:.3f}' for v in target_pos]}")
                for jn in finger_groups[selected_finger]:
                    jid_ = joint_id[jn]
                    cur_ = p.getJointState(robot_id, jid_)[0]
                    print(f"   {jn}: {cur_:+.3f} → {ik_dict[jn]:+.3f}")


        # ---- Animation interpolation ----
        if animating:
            current_step += 1
            t = min(current_step / float(animation_steps), 1.0)
            t_smooth = ease_in_out(t)

            for jid in start_poses:
                current_ang = start_poses[jid] + t_smooth * (target_poses[jid] - start_poses[jid])
                global_target_poses[jid] = current_ang

            if t >= 1.0:
                animating = False
                # In vị trí đạt được
                tip_lid = link_id[tip_link]
                actual = p.getLinkState(robot_id, tip_lid)[4]
                err = sum((a - b)**2 for a, b in zip(actual, target_pos)) ** 0.5
                print(f"   ✅ Xong!  Actual: ({actual[0]:+.3f}, {actual[1]:+.3f}, {actual[2]:+.3f})  "
                      f"Error: {err*1000:.1f} mm")

        # ---- Apply motor cho TẤT CẢ joints ----
        dt = 1.0 / SIM_HZ
        for jid, ang in global_target_poses.items():
            if CTRL_MODE == "position":
                p.setJointMotorControl2(robot_id, jid, p.POSITION_CONTROL,
                                        targetPosition=ang, force=JOINT_FORCE,
                                        positionGain=POS_GAIN)
            else:  # pi_velocity
                cur_ang = p.getJointState(robot_id, jid)[0]
                err = ang - cur_ang
                max_int = MAX_VEL / KI if KI > 0 else 1e9
                pi_integral[jid] = max(-max_int, min(max_int,
                                       pi_integral[jid] + err * dt))
                vel_cmd = max(-MAX_VEL, min(MAX_VEL,
                              KP * err + KI * pi_integral[jid]))
                p.setJointMotorControl2(robot_id, jid, p.VELOCITY_CONTROL,
                                        targetVelocity=vel_cmd, force=JOINT_FORCE)

        p.stepSimulation()
        time.sleep(1.0 / SIM_HZ)

except KeyboardInterrupt:
    p.disconnect()
    print("\n🔴 Dừng.")