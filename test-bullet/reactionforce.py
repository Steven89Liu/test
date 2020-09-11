import pybullet as p

physicsClient = p.connect(p.GUI)                                                                                                                                                                                                     

#p.setGravity(0, 0, -9.8)    # this sets the gravity

hinge = p.loadURDF("hinge.urdf")
hingeJointIndex = 0

p.setPhysicsEngineParameter(jointFeedbackMode=p.JOINT_FEEDBACK_IN_JOINT_FRAME)

p.setJointMotorControl2(hinge, hingeJointIndex,p.VELOCITY_CONTROL,0,0,0)
p.setGravity(0,0,0)

p.enableJointForceTorqueSensor(hinge,hingeJointIndex)

for _ in range(10000000):
    p.setJointMotorControl2(hinge,hingeJointIndex,p.TORQUE_CONTROL,force=10)
    #p.setJointMotorControl2(hinge,hingeJointIndex,p.POSITION_CONTROL,targetPosition=1.7, force=1000)
    p.stepSimulation()
    print(p.getJointState(hinge, hingeJointIndex)[0])
    print(p.getJointState(hinge, hingeJointIndex)[1])
    print(p.getJointState(hinge, hingeJointIndex)[2])
    print(p.getJointState(hinge, hingeJointIndex)[3])
    input()
