import pybullet as p
import time
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

planeId = p.loadURDF("plane.urdf")

# Sphere initial position is 3 meters high
sphere_start_pos = [0, 0, 3]

euler_angles = [0, 0, 0]

startOrientation = p.getQuaternionFromEuler(euler_angles)

sphere = p.loadURDF("sphere2.urdf", sphere_start_pos, startOrientation)

# Gravity value
g = -9.81

r = 0.5

y = sphere_start_pos[2]
v = 0
t = 1/240

for i in range (100000000):
    # Ecuations
    v = v + g*t
    y = y + v*t + 0.5*g*t**2

    # Check when the sphere reaches the ground
    if (y < r):
        continue

    # Update the position of the sphere
    p.resetBasePositionAndOrientation(sphere, [0, 0, y], [0, 0, 0, 1])

    p.stepSimulation()

    time.sleep(1 / 240)
