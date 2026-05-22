"""
Power Grasp — Large Cylinder
Tự động animate: MỞ → NẮM → GIỮ → MỞ (loop)
Không hiện thông số — chỉ visual.
"""
import pybullet as p
import pybullet_data
import time, os, math

exec(open(os.path.expanduser("~/bionic_hand_sim/fix_urdf.py")).read())
URDF = os.path.expanduser("~/bionic_hand_sim/assembly_1/urdf/assembly_1_fixed.urdf")

# ── Góc nắm tối ưu (đọc từ slider test) ───────────────────────────────────
GRASP = {
    "thumb_dmc_0":  0.915,
    "thumb_mcp_0":  1.268,
    "thumb_ip_0":   0.645,
    "index_mcp_0":  1.273,
    "index_pip_0":  1.529,
    "index_dip_0":  0.868,
    "middle_mcp_0": 1.281,
    "middle_pip_0": 1.472,
    "middle_dip_0": 0.901,
    "ring_mcp_0":   1.496,
    "ring_pip_0":   1.298,
    "ring_dip_0":   1.240,
    "little_mcp_0": 1.281,
    "little_pip_0": 1.133,
    "little_dip_0": 0.959,
}

negative_axis_joints = [
    "index_mcp_0", "little_mcp_0", "ring_mcp_0",
    "index_pip_0",  "middle_pip_0", "ring_dip_0", "thumb_ip_0",
]

SIM_HZ       = 240
ANIM_CLOSE_S = 2.0      # giây để đóng tay
ANIM_OPEN_S  = 1.5      # giây để mở tay
HOLD_S       = 2.5      # giây giữ nguyên sau khi nắm
OPEN_HOLD_S  = 0.8      # giây dừng ở tư thế mở trước khi nắm lại

# ── Connect ────────────────────────────────────────────────────────────────
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

p.resetDebugVisualizerCamera(
    cameraDistance=0.38,
    cameraYaw=42,
    cameraPitch=-32,
    cameraTargetPosition=[-0.02, 0.0, 0.20]
)

# Tắt grid / axes mặc định cho gọn
p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)          # ẩn panel PyBullet
p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 0)

p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"))

robot_id = p.loadURDF(
    URDF,
    basePosition=[0, 0, 0.15],
    baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
    useFixedBase=True
)

# ── Cylinder lớn (màu cam-nâu như ảnh) ────────────────────────────────────
cyl_col = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.025, height=0.12)
cyl_vis = p.createVisualShape(p.GEOM_CYLINDER, radius=0.025, length=0.12,
    rgbaColor=[0.8, 0.4, 0.1, 1.0])
cyl_id  = p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=cyl_col,
    baseVisualShapeIndex=cyl_vis,
    basePosition=[-0.04, 0.02, 0.195],
    baseOrientation=p.getQuaternionFromEuler([0, 1.5708, 0]),
)

# ── Joint maps ─────────────────────────────────────────────────────────────
name_to_id = {}
for i in range(p.getNumJoints(robot_id)):
    name_to_id[p.getJointInfo(robot_id, i)[1].decode()] = i

signs = {name_to_id[n]: (-1 if n in negative_axis_joints else 1)
         for n in GRASP if n in name_to_id}

grasp_targets = {name_to_id[n]: v for n, v in GRASP.items() if n in name_to_id}
open_targets  = {jid: 0.0 for jid in grasp_targets}

# ── Smooth ease-in-out ─────────────────────────────────────────────────────
def ease(t):
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)

def set_joints(pose_dict, force=6.0):
    for jid, val in pose_dict.items():
        p.setJointMotorControl2(
            robot_id, jid,
            p.POSITION_CONTROL,
            targetPosition=val * signs[jid],
            force=force,
            positionGain=0.3,
            velocityGain=1.0,
        )

def interp_pose(a, b, t):
    return {jid: a[jid] + (b[jid] - a[jid]) * t for jid in a}

# ── Trạng thái animation ───────────────────────────────────────────────────
# PHASE:  "open_hold" → "closing" → "hold" → "opening" → "open_hold" → ...
phase       = "open_hold"
phase_timer = 0.0
cur_pose    = dict(open_targets)   # góc hiện tại (để interpolate smooth)

PHASE_DUR = {
    "closing":   ANIM_CLOSE_S,
    "hold":      HOLD_S,
    "opening":   ANIM_OPEN_S,
    "open_hold": OPEN_HOLD_S,
}
PHASE_NEXT = {
    "open_hold": "closing",
    "closing":   "hold",
    "hold":      "opening",
    "opening":   "open_hold",
}

print("\n🟢 Cylinder Power Grasp — đang chạy animation loop.")
print("   Ctrl+C để dừng.\n")

loop = 0
snap_pose = dict(open_targets)   # pose tại thời điểm bắt đầu phase hiện tại

try:
    while True:
        loop      += 1
        phase_timer += 1.0 / SIM_HZ

        dur = PHASE_DUR[phase]
        t   = ease(phase_timer / dur)

        if phase == "closing":
            target = interp_pose(snap_pose, grasp_targets, t)
        elif phase == "opening":
            target = interp_pose(snap_pose, open_targets, t)
        else:                        # hold / open_hold
            target = cur_pose

        set_joints(target)
        cur_pose = target

        # Chuyển phase
        if phase_timer >= dur:
            snap_pose   = dict(cur_pose)
            phase       = PHASE_NEXT[phase]
            phase_timer = 0.0
            print(f"  → {phase}")

        p.stepSimulation()
        time.sleep(1.0 / SIM_HZ)

except KeyboardInterrupt:
    p.disconnect()
    print("\n🔴 Dừng.")
