import pybullet as p
import time

physicsClient = p.connect(p.GUI)
p.resetSimulation()
pos = [0,0,0]
orient = p.getQuaternionFromEuler([0,0,0])

robot = p.loadURDF("pendulum.urdf",basePosition=pos,baseOrientation = orient, useFixedBase=1)

p.resetJointState(robot, 0, 0.45)
p.setJointMotorControl2(robot,0,p.VELOCITY_CONTROL,force=0.000)
p.setJointMotorControl2(robot, 0, controlMode=p.TORQUE_CONTROL, force=0.000)

p.changeDynamics(robot, -1, linearDamping=0.0, angularDamping=0.0, contactStiffness=0.0, contactDamping=0.0, lateralFriction=0.0, spinningFriction=0.0, rollingFriction=0.0, jointDamping=0.0)
p.changeDynamics(robot, 0, linearDamping=0.0, angularDamping=0.0, contactStiffness=0.0, contactDamping=0.0, lateralFriction=0.0, spinningFriction=0.0, rollingFriction=0.0, jointDamping=0.0)
# p.changeDynamics(robot, 1, linearDamping=0.0, angularDamping=0.0, contactStiffness=0.0, contactDamping=0.0, lateralFriction=0.0, spinningFriction=0.0, rollingFriction=0.0, jointDamping=0.0)


print(p.getJointInfo(robot,0)[6])	# joint damping = 0.0
print(p.getJointInfo(robot,0)[7])	# joint friction = 0.0

p.setGravity(0, 0, -9.81)
p.setTimeStep(1./2400)

#for _ in range(100000):
while (1):
    p.stepSimulation()
    q = p.getJointState(robot, 0)[0]
    print(q)
    time.sleep(1 / 2400.)
    
p.disconnect()
