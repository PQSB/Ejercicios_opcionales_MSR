import pybullet as p
import time
import pybullet_data

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8) # Gravity on Earth

# For real time simulation
p.setTimeStep(1./240.)
p.setRealTimeSimulation(True)

planeId = p.loadURDF("plane.urdf")

euler_angles = [0, 0, 0]

euler_angles = [0, 0, 0]

startPosition = [0,0,1]
startOrientation = p.getQuaternionFromEuler(euler_angles)

robotId = p.loadURDF("3links.urdf",startPosition, startOrientation)

numJoints = p.getNumJoints(robotId)

for j in range(numJoints):
    print("%d - %s" % (p.getJointInfo(robotId, j)[0], p.getJointInfo(robotId, j)[1].decode("utf-8")))

joint_friction_id = p.addUserDebugParameter("jointFriction", 0, 10, 0)
joint_torque_id = p.addUserDebugParameter("joint_torque", -20, 20, -0.105)

# p.setJointMotorControl2(robotId, 1, p.VELOCITY_CONTROL, force=0)
p.changeDynamics(robotId, 1, localInertiaDiagonal=[0.0, 0.0, (1/3)*5*0.3])

for i in range (100000000):
    joint_friction = p.readUserDebugParameter(joint_friction_id)
    joint_torque = p.readUserDebugParameter(joint_torque_id)

    # Apply friction
    p.setJointMotorControl2(robotId, 1, p.VELOCITY_CONTROL, targetVelocity=0, force=joint_friction)
    
    # Apply torque
    p.setJointMotorControl2(robotId, 1, controlMode=p.TORQUE_CONTROL, force=joint_torque)

p.disconnect()
