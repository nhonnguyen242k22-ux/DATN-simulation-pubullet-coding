import pybullet as p
import pybullet_data
import os
import time

exec(open(os.path.expanduser("~/bionic_hand_sim/fix_urdf.py")).read())
URDF = os.path.expanduser("~/bionic_hand_sim/assembly_1/urdf/assembly_1_fixed.urdf")

p.connect(p.DIRECT) # direct for fast calculation
p.setAdditionalSearchPath(pybullet_data.getDataPath())
robot_id = p.loadURDF(
    URDF,
    basePosition=[0, 0, 0.15],
    useFixedBase=True
)

# Build maps
joint_id = {}
link_id = {}
for i in range(p.getNumJoints(robot_id)):
    info = p.getJointInfo(robot_id, i)
    joint_name = info[1].decode()
    link_name = info[12].decode()
    joint_id[joint_name] = i
    link_id[link_name] = i

THUMB_TIP = link_id.get("thumb_dp_2_1")
INDEX_TIP = link_id.get("dp_10_3_1")

# Move index finger a bit
p.resetJointState(robot_id, joint_id["index_mcp_0"], 0.5)
p.resetJointState(robot_id, joint_id["index_pip_0"], -0.5)

# Get index tip position
target_pos = p.getLinkState(robot_id, INDEX_TIP)[4] # worldLinkFramePosition

# Calculate IK for thumb to reach target_pos
joint_poses = p.calculateInverseKinematics(robot_id, THUMB_TIP, target_pos)

print("IK Returned:", joint_poses)

# We need to map the returned poses to the thumb joints.
# PyBullet returns poses for all *movable* joints (not fixed ones).
movable_joints = []
for i in range(p.getNumJoints(robot_id)):
    if p.getJointInfo(robot_id, i)[2] != p.JOINT_FIXED:
        movable_joints.append(p.getJointInfo(robot_id, i)[1].decode())

print("Movable joints:", movable_joints)
print("Thumb joint values:")
for i, name in enumerate(movable_joints):
    if "thumb" in name:
        print(f"  {name}: {joint_poses[i]}")

p.disconnect()
