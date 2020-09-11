#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2019 motion <motion@motion-MS-7971>
#
# Distributed under terms of the MIT license.

"""
Show the world
"""
from __future__ import print_function
import math
import time
import numpy as np
from klampt import WorldModel
from klampt.model import ik,coordinates,cartesian_trajectory,trajectory, create
from klampt.io import resource
from klampt.plan import cspace, robotplanning
from klampt.plan.robotplanning import makeSpace
from klampt import vis
from klampt.vis import GLSimulationPlugin, GLRealtimeProgram
from klampt.math import vectorops as vo
from klampt import Simulator, SimRobotController


def print_red(*args):
    print(*args)

world = WorldModel()
create.primitives.sphere(0.1, [-1, -1, 0], world=world, name='origin')
create.primitives.sphere(0.1, [0, -1, 0], world=world, name='X')
create.primitives.sphere(0.1, [-1, 1, 0], world=world, name='Y')
fn = "./ur5_plate.xml"
res = world.readFile(fn)
robot = world.robot(0)
space = makeSpace(world, robot)

path_data = np.load('../data/up_rrt_plans.npz')
plan = path_data['plan0']
q0 = plan[0]


class MyGLViewer(GLRealtimeProgram):
    def __init__(self, world, sim):
        GLRealtimeProgram.__init__(self, "GL test")
        self.world = world
        self.sim = sim
        self.controller = self.sim.controller(0)
        print('gains ', self.controller.getPIDGains())
        self.goal = None
        self.store_goal = None
        # add function to reset the world
        self.q_init = world.robot(0).getConfig()
        self.object_init = world.rigidObject(0).getTransform()

    def display(self):
        self.sim.updateWorld()
        self.world.drawGL()

    def idle(self):
        if self.goal is not None:
            self.controller.setMilestone(self.goal)
            print(self.controller.getPIDGains())
            self.goal = None
        self.sim.simulate(self.dt)
        if self.sim.getTime() % 3.0 <= 1e-6:
            self.reset()

    def set_goal(self, goal):
        self.goal = goal
        self.store_goal = goal

    def reset(self):
        print('reset')
        del self.sim
        world.robot(0).setConfig(self.q_init)
        world.rigidObject(0).setTransform(*self.object_init)
        self.sim = Simulator(world)
        self.controller = self.sim.controller(0)
        self.set_goal(self.store_goal)


def visualize(world, robot, path=None, start=None, goal=None):
    """Visualize a path"""
    # visualize start/goal as spheres if exist
    r = 0.04
    if start != None:
        sphere = create.primitives.sphere(r, start, world=world, name='start')
        # vis.add('start', sphere)
    if goal != None:
        create.primitives.sphere(r, goal, world=world, name='goal')

    vis.add("world", world)

    # animate path if exist
    if path != None:
        traj = trajectory.RobotTrajectory(robot, range(len(path)), path)
        vis.animate(("world", robot.getName()), path, speed=0.5)
        vis.add("trajectory", traj)

    vis.spin(float('inf'))
#visualize(world, robot, start=start,goal=goal)

qmin, qmax = robot.getJointLimits()
qmin[2] = -math.pi / 2
qmax[2] = 0
qmin[3] = 0
qmax[3] = math.pi
robot.setJointLimits(qmin, qmax)

config = robot.getConfig()
config[:] = q0[:len(config)].tolist()
robot.setConfig(config)
end_link = robot.link(6)  # for redundancy
end_link_pos = end_link.getWorldPosition([0, 0, 0])
object_pos = vo.add(end_link_pos, [0, 0.0, 0.18])
print('object at ', object_pos)
box = create.primitives.box(0.1, 0.1, 0.1, object_pos, world=world, name='box', mass=3)
print(type(box))
print(robot.getConfig())

path = np.zeros((plan.shape[0], len(config)))
path[:] = plan[:, :len(config)]
path = path.tolist()

# this allows visualization, but not real simulation
# visualize(world, robot, path.tolist())
sim = Simulator(world)
sim.setSetting("adaptiveTimeStepping", "0")
viewer = MyGLViewer(world, sim)
viewer.set_goal(path[-1])
viewer.run()
