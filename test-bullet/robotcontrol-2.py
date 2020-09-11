import pybullet as p
p.connect(p.GUI) #or p.SHARED_MEMORY or p.DIRECT

p.loadURDF("plane.urdf")
p.setGravity(0,0,-10)
huskypos = [0,0,0.1]

pole = p.loadURDF("cartpole.urdf", [1,1,0])

husky = p.loadURDF("husky/husky.urdf",huskypos[0],huskypos[1],huskypos[2])
numJoints = p.getNumJoints(husky)
for joint in range (numJoints) :
	print (p.getJointInfo(husky,joint))
targetVel = 10 #rad/s
maxForce = 100 #Newton

p.resetBaseVelocity(husky,[0.1,0,0],[0,0,0])
#for joint in range (2,6) :
#	p.setJointMotorControl(husky,joint,p.VELOCITY_CONTROL,targetVel,maxForce)

for step in range (30000):
    p.resetBaseVelocity(husky,[11,11,0],[0,0,0])
    p.stepSimulation()

targetVel=-2

#p.resetBaseVelocity(husky,[0,0,0],[0,0,0])
#for joint in range (2,6) :
#        p.setJointMotorControl(husky,joint,p.VELOCITY_CONTROL,targetVel,maxForce)
for step in range (40000):
    p.resetBaseVelocity(husky,[-0.11,-0.11,0],[0,0,0])
    p.stepSimulation()

p.getContactPoints(husky)

p.disconnect()


