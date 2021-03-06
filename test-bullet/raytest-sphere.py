import pybullet as p
import pybullet_data
import numpy as np

CLIENT = p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF

def test_smallsphere(pos_): 
    pos = np.array(pos_)
    ss = p.loadURDF("sphere_small.urdf")
    p.resetBasePositionAndOrientation(ss, pos, [0, 0, 0, 1])
    margin = np.array([0.3, 0.0, 0.0])
    p_start = pos - margin
    p_end = pos + margin
    p.addUserDebugLine(p_start, p_end, [1, 0, 0])
    result = p.rayTest(p_start, p_end)
    return result

result = test_smallsphere([0.0, 0, 0.000])
print(result)
input()
