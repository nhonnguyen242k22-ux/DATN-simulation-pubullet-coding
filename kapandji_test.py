import pybullet as p
import pybullet_data
import time
import os

exec(open(os.path.expanduser("~/bionic_hand_sim/fix_urdf.py")).read())
URDF = os.path.expanduser("~/bionic_hand_sim/assembly_1/urdf/assembly_1_fixed.urdf")

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.resetDebugVisualizerCamera(
    cameraDistance=0.5, cameraYaw=45,
    cameraPitch=-45, cameraTargetPosition=[0, 0, 0.1],
)

p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"))
robot_id = p.loadURDF(
    URDF,
    basePosition=[0, 0, 0.15],
    baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
    useFixedBase=True
)

negative_axis_joints = [
    "index_mcp_0", "little_mcp_0", "ring_mcp_0",
    "index_pip_0", "middle_pip_0", "ring_dip_0", "thumb_ip_0"
]

# Build joint and link maps
joint_name_to_id = {}
link_name_to_id = {}
movable_joints = []
for i in range(p.getNumJoints(robot_id)):
    info = p.getJointInfo(robot_id, i)
    jname = info[1].decode()
    lname = info[12].decode()
    joint_name_to_id[jname] = i
    link_name_to_id[lname] = i
    if info[2] != p.JOINT_FIXED:
        movable_joints.append(i)

THUMB_TIP = link_name_to_id.get("thumb_dp_2_1")

# Child link names confirmed from URDF parent/child structure
KAPANDJI_TARGET_LINKS = {
    1:  "mcp_10_1_2",   # index proximal phalanx
    2:  "pip_10_2",     # index middle phalanx
    3:  "dp_10_3_1",    # index tip
    4:  "dp_10_3_3",    # middle tip
    5:  "dp_10_3_2",    # ring tip
    6:  "dp_10_3",      # little tip
    7:  "pip_10_2_1",   # little middle phalanx — DIP crease
    8:  "mcp_10_1_1",   # little proximal phalanx — PIP crease
    9:  "dmc_2",        # little metacarpal — MCP crease
    10: "palm_3",       # palm — palmar crease
}

score_slider = p.addUserDebugParameter("Kapandji Score", 0, 10, 0)
txt_score = p.addUserDebugText("Score: 0", [0, 0, 0.45], textColorRGB=[1, 1, 1], textSize=1.5)

print("\n🟢 Kapandji IK test ready (left hand). Set slider to 1-10. Ctrl+C to quit.\n")

prev_score = -1
prev_target_link = None
original_color = [0.8, 0.8, 0.8, 1.0]

try:
    while True:
        score = int(round(p.readUserDebugParameter(score_slider)))

        if score != prev_score:
            # Restore color
            if prev_target_link is not None:
                p.changeVisualShape(robot_id, prev_target_link, rgbaColor=original_color)
            
            p.addUserDebugText(f"Score: {score}", [0, 0, 0.45], textColorRGB=[1, 1, 1], textSize=1.5, replaceItemUniqueId=txt_score)

            if score == 0:
                # Neutral pose
                for i in movable_joints:
                    p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, 0, force=5.0)
                prev_target_link = None
            else:
                target_link_name = KAPANDJI_TARGET_LINKS[score]
                target_link = link_name_to_id.get(target_link_name)
                
                if target_link is not None:
                    # Highlight target
                    p.changeVisualShape(robot_id, target_link, rgbaColor=[1.0, 1.0, 0.0, 1.0])
                    prev_target_link = target_link

                    # Bend fingers
                    for name, jid in joint_name_to_id.items():
                        if "thumb" not in name and jid in movable_joints:
                            sign = -1 if name in negative_axis_joints else 1
                            target_val = 0.3 * sign
                            p.setJointMotorControl2(robot_id, jid, p.POSITION_CONTROL, targetPosition=target_val, force=5.0)

                    # Give it a few steps to reach there
                    for _ in range(50):
                        p.stepSimulation()

            prev_score = score
        
        # Continuously update IK for thumb if score > 0
        if score > 0 and prev_target_link is not None:
            target_pos = p.getLinkState(robot_id, prev_target_link)[4]
            ik_poses = p.calculateInverseKinematics(robot_id, THUMB_TIP, target_pos)

            for i, jid in enumerate(movable_joints):
                name = p.getJointInfo(robot_id, jid)[1].decode()
                if "thumb" in name:
                    p.setJointMotorControl2(robot_id, jid, p.POSITION_CONTROL, targetPosition=ik_poses[i], force=5.0)

            # Check contact
            thumb_pos = p.getLinkState(robot_id, THUMB_TIP)[4]
            dist = sum((a - b) ** 2 for a, b in zip(thumb_pos, target_pos)) ** 0.5
            if dist < 0.015:
                p.changeVisualShape(robot_id, prev_target_link, rgbaColor=[0.0, 1.0, 0.0, 1.0])
            else:
                p.changeVisualShape(robot_id, prev_target_link, rgbaColor=[1.0, 1.0, 0.0, 1.0])

        p.stepSimulation()
        time.sleep(1.0 / 240.0)

except KeyboardInterrupt:
    p.disconnect()
    print("\n🔴 Done.")
