import pybullet as p
import pybullet_data

# Connect to the GUI simulation
p.connect(p.GUI)

# (Optional) Add the default pybullet_data path so you can load basic shapes like planes
p.setAdditionalSearchPath(pybullet_data.getDataPath())
# Set gravity pointing down
p.setGravity(0, 0, -9.81)

# Load a default ground plane
p.loadURDF("plane.urdf") 
# Path to your URDF file
urdf_path = "/home/nhon/bionic_hand_sim/assembly_1/urdf/assembly_1.urdf"

# Import the URDF into the simulation
robot_id = p.loadURDF(
    fileName=urdf_path,
    basePosition=[0, 0, 0.15], # [x, y, z] coordinates to spawn the robot
    baseOrientation=p.getQuaternionFromEuler([0, 0, 0]), # Rotation
    useFixedBase=True # Set to True if the robot is bolted to the ground/world
)
import time

# Create sliders for all joints
sliders = {}
for i in range(p.getNumJoints(robot_id)):
    joint_info = p.getJointInfo(robot_id, i)
    joint_name = joint_info[1].decode('utf-8')
    joint_type = joint_info[2]
    
    # Check if joint is movable (revolute = 0, prismatic = 1)
    if joint_type in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:
        # addUserDebugParameter(name, rangeMin, rangeMax, startValue)
        # Starting at zero as requested
        sliders[i] = p.addUserDebugParameter(joint_name, -3.14, 3.14, 0.0)

print("\n🟢 Use the sliders in the PyBullet GUI to control the joints. Press Ctrl+C to stop in terminal.")

try:
    while True:
        # Update joint positions based on sliders
        for jid, slider_id in sliders.items():
            target_pos = p.readUserDebugParameter(slider_id)
            p.setJointMotorControl2(
                bodyUniqueId=robot_id,
                jointIndex=jid,
                controlMode=p.POSITION_CONTROL,
                targetPosition=target_pos,
                force=5.0
            )
            
        # Step the physics engine forward
        p.stepSimulation()
        # Sleep to match real-time
        time.sleep(1./240.)
except KeyboardInterrupt:
    # Disconnect cleanly when you press Ctrl+C
    p.disconnect()
    print("\n🔴 Stopped.")
