import pybullet as p
import time

p.connect(p.GUI)
p.setPhysicsEngineParameter(enableFileCaching=0)

p.loadURDF("plane.urdf")
p.setGravity(0, 0, -9.81)

p.loadURDF(
    fileName="shaft1.urdf", basePosition=(0, 0, 1), baseOrientation=(0, 0, 0, 1),
    flags=p.URDF_USE_SELF_COLLISION | p.URDF_USE_INERTIA_FROM_FILE
)
p.loadURDF(
    fileName="shaft1.urdf", basePosition=(0, 1, 1), baseOrientation=(1, 0, 0, 1),
    flags=p.URDF_USE_SELF_COLLISION | p.URDF_USE_INERTIA_FROM_FILE
)
p.loadURDF(
    fileName="shaft1.urdf", basePosition=(1, 0, 1), baseOrientation=(0, 1, 0, 1),
    flags=p.URDF_USE_SELF_COLLISION | p.URDF_USE_INERTIA_FROM_FILE
)
p.loadURDF(
    fileName="shaft1.urdf", basePosition=(1, 1, 1), baseOrientation=(0, 0, 1, 1),
    flags=p.URDF_USE_SELF_COLLISION | p.URDF_USE_INERTIA_FROM_FILE
)

for t in range(100000):
    time.sleep(1./240)
    p.stepSimulation()


