import pybullet as p
import time
import pybullet_data
import math

#joint numbers

#setup pybullet
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.81)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,2]
#cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
#robotId = p.loadURDF("closedKinematicLoop.urdf",basePosition =cubeStartPos,baseOrientation=cubeStartOrientation, useFixedBase=True)#,flags=URDF_USE_INERTIA_FROM_FILE)
robotId = p.loadURDF("TwoJointRobot_w_fixedJoints2.urdf",basePosition =cubeStartPos, useFixedBase=True, globalScaling=1.0)#,flags=2)
#cid = p.createConstraint(planeId,-1,robotId,-1,p.JOINT_PRISMATIC,[0,0,1],[0,0,0],[0,0,0])
#for j in range(3):
#    p.setJointMotorControl2(robotId,j,p.VELOCITY_CONTROL,0,0,0)
#cPanto= p.createConstraint(robotId,-1,robotId,2,p.JOINT_POINT2POINT,jointAxis=[0,0,0],parentFramePosition=[0,0,0],childFramePosition=[0,0,-0.15])
while 1:
    p.stepSimulation()
    print("test...")
    #p.setJointMotorControl2(robotId,0,p.POSITION_CONTROL,0)
  #  p.changeConstraint(cid,maxForce=10)
    time.sleep(1./240.) #default 240
p.disconnect()
