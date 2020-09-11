import pybullet as p
import time
import numpy as np

import os
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

class AllegroHand:
    def __init__(self,
                 base_init_pos=np.array([0., 0, 0.5]),
                 base_init_euler=np.array([1.57, 0, 0])):
        self.baseInitPos = base_init_pos
        self.baseInitOri = base_init_euler

        self.handId = p.loadURDF(os.path.join(currentdir, "assets/allegro_hand_description/allegro_hand_description_right.urdf"),
                                 list(self.baseInitPos), p.getQuaternionFromEuler(list(self.baseInitOri)),
                                 flags=p.URDF_USE_SELF_COLLISION)

        self.savedState = p.saveState()

    def reset(self):
        p.restoreState(self.savedState)

    def get_raw_state_fingers(self):
        joints_state = p.getJointStates(self.handId, range(20))
        joints_state = np.array(joints_state)[:,[0,1]]    # only position, no vel.
        return np.hstack(joints_state.flatten())

    def get_robot_observation(self):
        obs = []
        obs.extend(list(self.get_raw_state_fingers()))
        basePos, baseQuat = p.getBasePositionAndOrientation(self.handId)
        obs.extend(basePos)
        obs.extend(baseQuat)
        return obs

    def get_robot_observation_dim(self):
        return len(self.get_robot_observation())

    def apply_action(self, a):
        # a is not used
        for i in list([5,6,7,8,10,11,12,13,15,16,17,18]):
            p.setJointMotorControl2(self.handId,
                                    jointIndex=i,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=-1.0,
                                    force=300.0)


if __name__ == "__main__":
    physicsClient = p.connect(p.GUI)

    p.setTimeStep(1./240)
    p.setGravity(0, 0, -10)
    p.loadURDF(os.path.join(currentdir, 'assets/plane.urdf'), [0, 0, 0], useFixedBase=1)

    a = AllegroHand()

    for i in range(100):
        a.reset()

        input("press enter to continue")
        print("init", a.get_robot_observation())
        for t in range(500):
            a.apply_action(np.array([np.nan]*22))
            p.stepSimulation()
            time.sleep(1./240.)
        print("final obz", a.get_robot_observation())

    p.disconnect()
