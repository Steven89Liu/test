#!/usr/bin/python
# 
# rbdl mass matrix
import rbdl
import numpy as np
import csv

np.set_printoptions(precision=6, suppress=True)
mdl = rbdl.loadModel(b'/home/shjliu/workspace/EPR/bullet3-new/20200214/bullet3/examples/pybullet/gym/pybullet_data/franka_panda/panda.urdf')
mass_matrix = np.zeros((9, 9), dtype=np.double)
#neutral_joint_angles= [0., -0., 0., -0, 0., 0, 0.]
neutral_joint_angles= [0., -0.3135, 0., -2.515, 0., 2.226, 0.87]

rbdl.CompositeRigidBodyAlgorithm(mdl, 
		np.asarray(neutral_joint_angles, dtype=np.double),
		mass_matrix) 
print("mass_matrix=")
print(mass_matrix)
