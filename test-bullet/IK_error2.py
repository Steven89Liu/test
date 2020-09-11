import pybullet as p
import math
import pybullet_data

clid = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally

kukaId = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0])
p.resetBasePositionAndOrientation(kukaId, [0, 0, 0], [0, 0, 0, 1])
kukaEndEffectorIndex = 6
numJoints = p.getNumJoints(kukaId)

#lower limits for null space
#ll = [-.967, -2, -2.96, 0.19, -2.96, -2.09, -3.05]
#upper limits for null space
#ul = [.967, 2, 2.96, 2.09, 2.96, 2.09, 3.05]


#lower limits for null space
ll = [-.967, -2, -2.96, 0.19, -2.96, -2.09, -3.05]
#upper limits for null space
ul = [.967, 2, 2.96, 2.09, 2.96, 2.09, 3.05]

#joint ranges for null space
jr = [5.8, 4, 5.8, 4, 5.8, 4, 6]
#restposes for null space
rp = [0, 0, 0, 0.5 * math.pi, 0, -math.pi * 0.5 * 0.66, 0]
#joint damping coefficents
jd = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

for i in range(numJoints):
  p.resetJointState(kukaId, i, rp[i])


pos = [-0.1, 0, 0.2]
#end effector points down, not up (in case useOrientation==1)
orn = p.getQuaternionFromEuler([0, -math.pi, 0])

jointPoses = p.calculateInverseKinematics(kukaId, kukaEndEffectorIndex, pos, orn, ll, ul, jr, rp, maxNumIterations=100)
#jointPoses = p.calculateInverseKinematics(kukaId, kukaEndEffectorIndex, pos, lowerLimits=ll, upperLimits=ul, jointRanges=jr, restPoses=rp, maxNumIterations=100)
print("jonintPoses = ", jointPoses)
