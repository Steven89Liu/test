import os
import math
import numpy as np
import time
import requests

import pybullet as p
import pybullet_data

from scipy.spatial.transform import Rotation as R


joint_ll = [-3, -3, -10, 0, -1.57, -10, 0, -10]
joint_ul = [3, 3, 10, 2, 0, 10, 1.57, 10]
joint_jr = [u - l for l, u in zip(joint_ll, joint_ul)]


def move_bot(botId, goal_pos, goal_dir, steps=500, reset=False):
    goalJoints = p.calculateInverseKinematics(botId, 9, goal_pos, targetOrientation=goal_dir,
            lowerLimits=joint_ll,
            upperLimits=joint_ul,
            jointRanges=joint_jr,
            restPoses=[0, 0, 0, 1, -0.1, 0, 0, 0.1, 0],
            jointDamping=[0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001])

    numJoints = p.getNumJoints(botId)

    def set_joints(reset_joints):
        for ji in range(numJoints):
            jointInfo = p.getJointInfo(botId, ji)
            qIndex = jointInfo[3]

            if qIndex > -1:
                x = goalJoints[qIndex-7]

                if qIndex-7 < len(joint_ll) and (x < joint_ll[qIndex-7] or x > joint_ul[qIndex-7]):
                    print('LIMIT VIOLATION!!', x, joint_ll[qIndex-7], joint_ul[qIndex-7], jointInfo[1])
                    #exit()

                if reset_joints:
                    p.resetJointState(botId, ji, x, 0)

                p.setJointMotorControl2(botId, ji, p.POSITION_CONTROL, targetPosition=x, targetVelocity=0,
                            force=500,
                            positionGain=0.03,
                            velocityGain=1)

    def check_joints(goalJoints):
        jointStates = []
        jointNames = []

        if goalJoints is not None:
            numJoints = p.getNumJoints(botId)

            for ji in range(numJoints):
                jointInfo = p.getJointInfo(botId, ji)
                qIndex = jointInfo[3]

                if qIndex > -1:
                    x = goalJoints[qIndex-7]

                    jointPos = p.getJointState(botId, ji)[0]

                    jointNames.append(jointInfo[1])
                    jointStates.append(jointPos)

        return jointStates, jointNames

    set_joints(False)

    stats = [[] for _ in range(len(goalJoints))]

    for i in range(steps):
        if reset:
            print('setting joints')
            set_joints(True)

        #for _ in range (10):
        p.stepSimulation()
        
        if i % 10 == 0:
            pos, names = check_joints(goalJoints)

            for pi, px in enumerate(pos):
                stats[pi].append(px)
    print("matplotlib")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 10)

    for i, (g, s) in enumerate(zip(goalJoints, stats)):
        ax[i].set_title(str(names[i]))
        ax[i].plot(s, label='actual')
        ax[i].plot([0, 100], [g, g], label='target')
        if i < len(joint_ul):
            ax[i].plot([0, 100], [joint_ll[i], joint_ll[i]], label='lower')
            ax[i].plot([0, 100], [joint_ul[i], joint_ul[i]], label='upper')
    fig.set_size_inches(30, 2)
    fig.savefig('joints.png')
    #fig.show()
def resetJoints(botId):
    user_x_pos = 0
    user_y_pos = 0
    user_t_pos = 0.628
    user_lift = 1
    user_arm = 0
    user_arm_roll = 4.495
    user_wrist_flex = 1.57
    user_wrist_roll = 0
    user_gripper_width = 0

    # p.resetJointState(botId, 0, user_x_pos)
    # p.resetJointState(botId, 1, user_y_pos)
    # p.resetJointState(botId, 2, user_t_pos)
    # p.resetJointState(botId, 4, user_lift)
    # p.resetJointState(botId, 5, user_arm)
    # p.resetJointState(botId, 6, user_arm_roll)
    # p.resetJointState(botId, 7, user_wrist_flex)
    # p.resetJointState(botId, 9, user_wrist_roll)
    # p.resetJointState(botId, 10, user_gripper_width / 2.0)
    # p.resetJointState(botId, 11, user_gripper_width / 2.0)

    p.setJointMotorControlArray(botId, [0, 1, 2, 4, 5, 6, 7, 9, 10, 11],
        p.POSITION_CONTROL, [user_x_pos, user_y_pos, user_t_pos, user_lift,
        user_arm, user_arm_roll, user_wrist_flex, user_wrist_roll,
        0, 0])

def runSim():
    p.setGravity(0,0,-10)
    p.setTimeStep(0.01)
    #planeId = p.loadURDF("plane.urdf")
    botId = p.loadURDF("bot.xml",
                      [0, 0, 0.001],
                      p.getQuaternionFromEuler([0,0,0]))

    vs_id = p.createVisualShape(p.GEOM_SPHERE, radius=0.05, rgbaColor=[0, 1, 0, 1])
    marker_id = p.createMultiBody(basePosition=[0, 0, 0], baseCollisionShapeIndex=-1, baseVisualShapeIndex=vs_id)
    vs_id = p.createVisualShape(p.GEOM_SPHERE, radius=0.05, rgbaColor=[0, 0, 1, 1])
    marker2_id = p.createMultiBody(basePosition=[0, 0, 0], baseCollisionShapeIndex=-1, baseVisualShapeIndex=vs_id)

    resetJoints(botId)

    pos = [np.random.uniform(-0.5, 0.5), np.random.uniform(-0.5, 0.5), 1.0]
    ori = p.getQuaternionFromEuler([np.random.uniform(-np.pi, np.pi),
        np.random.uniform(-np.pi, np.pi), np.random.uniform(-np.pi, np.pi)])
    T = R.from_quat(ori).as_matrix()

    p.resetBasePositionAndOrientation(marker_id, pos, ori)
    p.resetBasePositionAndOrientation(marker2_id, pos + T[:3, 2] * 0.05, ori)

    for _ in range(300):
        p.stepSimulation()

    move_bot(botId, pos, ori)
    time.sleep(1)
    move_bot(botId, pos, ori, reset=True)
    time.sleep(1)
    move_bot(botId, pos, ori)

if __name__ == '__main__':
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    np.random.seed(0)

    for i in range(10000000000000):
        print('new trial:', i)
        p.resetSimulation()
        runSim()

