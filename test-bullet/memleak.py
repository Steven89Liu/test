import os
import psutil
import pybullet
from pybullet_envs.bullet.kuka_diverse_object_gym_env import KukaDiverseObjectEnv  # NOQA


env = KukaDiverseObjectEnv()
process = psutil.Process(os.getpid())
pybullet.setPhysicsEngineParameter(enableFileCaching=0)

for i in range(50):
    env.reset()
    print(i, process.memory_info().rss)
