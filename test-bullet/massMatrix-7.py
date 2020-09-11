import pybullet as pb
import numpy as np

physics_id = pb.connect(pb.GUI)
np.set_printoptions(precision=6, suppress=True)

arm_id = pb.loadURDF(
    fileName='/home/shjliu/workspace/EPR/bullet3-new/20200214/bullet3/examples/pybullet/gym/pybullet_data/franka_panda/panda-only-7.urdf',
#     fileName='./balance.urdf',
    basePosition=[0,0,0],
    baseOrientation=[0,0,0,1],
    useFixedBase=True,
    flags=pb.URDF_USE_INERTIA_FROM_FILE,
    physicsClientId=physics_id)

pb.setGravity(0, 0, -9.8, physicsClientId=physics_id)

#neutral_joint_angles= [0., -0.3135, 0., -2.515, 0., 2.226, 0.87]
neutral_joint_angles= [0., -0., 0., -0, 0., 0, 0.]

for index, angle in enumerate(neutral_joint_angles):
    pb.resetJointState(
        bodyUniqueId=arm_id, 
        jointIndex=index, 
        targetValue=angle, 
        physicsClientId=physics_id)

pb.stepSimulation(physics_id)

joint_positions = []
for joint_index in range(9):
    joint_positions.append(pb.getJointState(
        arm_id, 
        joint_index, 
        physics_id)[0])

print("joint positions")
print(joint_positions)

mass_matrix = pb.calculateMassMatrix(
            bodyUniqueId=arm_id,
            objPositions=joint_positions,
            physicsClientId=physics_id)

print("Mass Matrix")
tmp = np.array(mass_matrix)
print(tmp)
while 1:
   pb.stepSimulation()
#   print("hello")
