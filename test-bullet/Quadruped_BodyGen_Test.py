import pybullet as p
import time
import pybullet_data

physicsClient = p.connect(p.GUI) # or p.DIRECT for non graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
#p.setTimeStep(1.0)
planeID = p.loadURDF("plane.urdf")

p.resetDebugVisualizerCamera( cameraDistance=0.5, cameraYaw=75, cameraPitch=-20, cameraTargetPosition=[0.0, 0.0, 0.25])

baseStartPos = [0,0,0.5]
baseStartOrientation = p.getQuaternionFromEuler([1,0,0])
quadrupedID = p.loadURDF("MQ_SIM_ASEM/urdf/MQ_SIM_ASEM.urdf",baseStartPos, baseStartOrientation)

#p.changeDynamics(sphereUid,-1,spinningFriction=0.001, rollingFriction=0.001,linearDamping=0.0)
# Set Joints to home position
for joint in range (p.getNumJoints(quadrupedID)):
	p.setJointMotorControl2(quadrupedID,joint,p.POSITION_CONTROL,targetPosition=0.0,force=10,maxVelocity=5)


# Set Shoulder Joints	
joint = 0	
p.setJointMotorControl2(quadrupedID,joint,p.POSITION_CONTROL,targetPosition=-0.125,force=5,maxVelocity=5)
joint = 3	
p.setJointMotorControl2(quadrupedID,joint,p.POSITION_CONTROL,targetPosition=0.125,force=5,maxVelocity=5)
joint = 6	
p.setJointMotorControl2(quadrupedID,joint,p.POSITION_CONTROL,targetPosition=-0.125,force=5,maxVelocity=5)
joint = 9	
p.setJointMotorControl2(quadrupedID,joint,p.POSITION_CONTROL,targetPosition=0.125,force=5,maxVelocity=5)	

# Set Upper Joints	
joint = 1	
p.setJointMotorControl2(quadrupedID,joint,p.POSITION_CONTROL,targetPosition=-0.75,force=5,maxVelocity=5)
joint = 4	
p.setJointMotorControl2(quadrupedID,joint,p.POSITION_CONTROL,targetPosition=-0.75,force=5,maxVelocity=5)
joint = 7	
p.setJointMotorControl2(quadrupedID,joint,p.POSITION_CONTROL,targetPosition=-0.75,force=5,maxVelocity=5)
joint = 10	
p.setJointMotorControl2(quadrupedID,joint,p.POSITION_CONTROL,targetPosition=-0.75,force=5,maxVelocity=5)

# Set Lower Joints	
joint = 2	
p.setJointMotorControl2(quadrupedID,joint,p.POSITION_CONTROL,targetPosition=0.2,force=100,maxVelocity=5)
joint = 5	
p.setJointMotorControl2(quadrupedID,joint,p.POSITION_CONTROL,targetPosition=0.2,force=100,maxVelocity=5)
joint = 8	
p.setJointMotorControl2(quadrupedID,joint,p.POSITION_CONTROL,targetPosition=0.2,force=100,maxVelocity=5)
joint = 11	
p.setJointMotorControl2(quadrupedID,joint,p.POSITION_CONTROL,targetPosition=0.2,force=100,maxVelocity=5)

#Set Link Dynamics
joint = 2
p.changeDynamics(quadrupedID,joint,lateralFriction=0.1,spinningFriction=0.001, rollingFriction=0.001)
joint = 5
p.changeDynamics(quadrupedID,joint,lateralFriction=0.1,spinningFriction=0.001, rollingFriction=0.001)
joint = 8
p.changeDynamics(quadrupedID,joint,lateralFriction=0.1,spinningFriction=0.001, rollingFriction=0.001)
joint = 11
p.changeDynamics(quadrupedID,joint,lateralFriction=0.1,spinningFriction=0.001, rollingFriction=0.001)

p.getNumJoints(quadrupedID)
for i in range (p.getNumJoints(quadrupedID)):
	print(p.getJointInfo(quadrupedID,i))
	
p.setGravity(0,0,-9.81)
p.setRealTimeSimulation(1)

while (1):
	keys = p.getKeyboardEvents()
	#print(keys)
	basePos, baseOrn = p.getBasePositionAndOrientation(quadrupedID) # Get model position
	p.resetDebugVisualizerCamera( cameraDistance=0.3, cameraYaw=75, cameraPitch=-20, cameraTargetPosition=basePos) # fix camera onto model
	time.sleep(0.01)

basePos, baseOrn = p.getBasePositionAndOrientation(quadrupedID)
print(basePos, baseOrn)
p.disconnect()