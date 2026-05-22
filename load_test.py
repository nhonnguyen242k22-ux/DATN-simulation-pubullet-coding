import pybullet as p
import pybullet_data
import time, os

exec(open(os.path.expanduser("~/bionic_hand_sim/fix_urdf.py")).read())

URDF = os.path.expanduser(
    "~/bionic_hand_sim/assembly_1/urdf/assembly_1_fixed.urdf"
)

# Joints có axis "-1 0 0" và limit âm → slider × (−1) trước khi gửi PyBullet
# Thêm index_mcp, little_mcp, ring_mcp vì URDF mới đổi axis sang -1 0 0
negative_axis_joints = [
    "index_mcp_0",
    "little_mcp_0",
    "ring_mcp_0",
    "index_pip_0",
    "middle_pip_0",
    "ring_dip_0",
    "thumb_ip_0",
]

custom_ranges = {
    "thumb_dmc_0": (-0.5236, 2.6179, 0.0),  # -30° to 150°
    "thumb_mcp_0": (-0.5236, 2.0944, 0.0),  # -30° to 120°
}

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.resetDebugVisualizerCamera(
    cameraDistance=0.5,
    cameraYaw=45,
    cameraPitch=-45,
    cameraTargetPosition=[0, 0, 0.1]
)

p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"))
robot_id = p.loadURDF(
    URDF,
    basePosition=[0, 0, 0.15],
    baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
    useFixedBase=True
)

name_to_id = {}
for i in range(p.getNumJoints(robot_id)):
    name = p.getJointInfo(robot_id, i)[1].decode()
    name_to_id[name] = i

finger_groups = {
    "Thumb":  ["thumb_dmc_0", "thumb_mcp_0", "thumb_ip_0"],
    "Index":  ["index_mcp_0", "index_pip_0", "index_dip_0"],
    "Middle": ["middle_mcp_0", "middle_pip_0", "middle_dip_0"],
    "Ring":   ["ring_mcp_0",   "ring_pip_0",  "ring_dip_0"],
    "Little": ["little_mcp_0", "little_pip_0", "little_dip_0"],
}

sliders = {}
signs   = {}

for finger, joints in finger_groups.items():
    for jname in joints:
        if jname in name_to_id:
            jid = name_to_id[jname]
            label = f"{finger} {jname.split('_')[1].upper()}"
            if jname in custom_ranges:
                lo, hi, default = custom_ranges[jname]
            else:
                lo, hi, default = 0.0, 1.5708, 0.0
            sliders[jid] = p.addUserDebugParameter(label, lo, hi, default)
            signs[jid]   = -1 if jname in negative_axis_joints else 1

print("\n🟢 Chạy! Kéo slider 0→1.57 = gập ngón. Ctrl+C dừng.")

try:
    while True:
        for jid, slider_id in sliders.items():
            val  = p.readUserDebugParameter(slider_id)
            sign = signs[jid]
            p.setJointMotorControl2(
                bodyUniqueId=robot_id,
                jointIndex=jid,
                controlMode=p.POSITION_CONTROL,
                targetPosition=val * sign,
                force=5.0
            )
        p.stepSimulation()
        time.sleep(1./240.)
except KeyboardInterrupt:
    p.disconnect()
    print("\n🔴 Dừng.")
