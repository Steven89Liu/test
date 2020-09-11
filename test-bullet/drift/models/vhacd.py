import pybullet as p
import pybullet_data as pd
import os
import time

p.connect(p.DIRECT)
dir="data/"
name_in = "crane.obj"
name_out = "crane_vhacd.obj"
name_log = "log.txt"

#for more parameters, see http://pybullet.org/guide

p.vhacd(name_in, name_out, name_log, resolution=1000000, depth=20)

