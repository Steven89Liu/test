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
                                 list(self.baseInitPos), p.getQuaternionFromEuler(list(self.baseInitOri)), flags=p.URDF_USE_SELF_COLLISION)

        self.savedState = p.saveState()

    def reset(self):
        p.restoreState(self.savedState)

    def get_raw_state_fingers(self):
        joints_state = p.getJointStates(self.handId, range(20))
        joints_state = np.array(joints_state)[:,[0]]    # only position, no vel.
        print("joints_state=", joints_state)
        return np.hstack(joints_state.flatten())

    def get_robot_observation(self):
        obs = []
        obs.extend(list(self.get_raw_state_fingers()))
        basePos, baseQuat = p.getBasePositionAndOrientation(self.handId)
        print("basePos=", basePos, "baseQuat=", baseQuat)
        obs.extend(basePos)
        #obs.extend(baseQuat)
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
    old = []
    new = []
    first = []
    count = 0
    for i in range(100):
        a.reset()
        #p.resetSimulation()
        #if count >= 0:
        #    a.reset()
        input("press enter to continue")
        print('')
        print("init", a.get_robot_observation())
        a.apply_action(np.array([np.nan]*22))
        for t in range(500):
            a.apply_action(np.array([np.nan]*22))
            p.stepSimulation()
            basePos, baseQuat = p.getBasePositionAndOrientation(a.handId)
            #print("basePos=", basePos, "loop index=", t)
            #time.sleep(1./240.)
        new = a.get_robot_observation()
        print("final obz", new, "new len =", len(new))
        if count == 0:
            first = new
            old = first
            count = count + 1
        max1 = [new[i] - old[i] for i in range(len(new))]
        max2 = max(max1)
        indexmax = max1.index(max2)
        min1 = [new[i] - old[i] for i in range(len(new))]
        min2 = min(min1)
        indexmin = min1.index(min2)
        print("max=", max2, "index=", indexmax, "min=", min2, "indexmin=", indexmin)
    p.disconnect()
