# E. Culurciello
# March 2021

# test for desk environment
# based on: https://github.com/google-research/google-research/tree/master/playrooms

import os, sys 
import time
import pybullet
import pybullet_data
from collections import namedtuple
from attrdict import AttrDict
import numpy as np
from gym_env import DeskGymEnv


# testing usage:
if __name__ == '__main__':
    # initialize env:
    env = DeskGymEnv(renders=True)

    # pos is: [x, y, z, gripper] gripper > 0 is closed, < 0 is open

    pos = [0., 0., 0.9, 1.0] # default pose + open gripper
    env.step(pos)
    time.sleep(1)


    pos = [-0.0, -0.0, 0.45, 0.0] # reach drawer + closed gripper
    env.step(pos)
    time.sleep(1)

    pos = [-0.0, 0.1, 0.45, 0.0] # open drawer
    env.step(pos)
    time.sleep(1)

    pos = [0., 0., 0.9, 1.0] # default pose + open gripper
    env.step(pos)
    time.sleep(1)

    pos = [-0.4, -0.4, 0.6, 0.0] # reach slide
    env.step(pos)
    time.sleep(1)

    pos = [0., 0., 0.9, 1.0] # default pose 
    env.step(pos)
    time.sleep(1)


    pos = [0.25, -0.35, 0.7, 0.0] # reach buttons
    env.step(pos)
    time.sleep(1)
 
    pos = [0., 0., 0.9, 1.0] # default pose 
    env.step(pos)
    time.sleep(1) 

    pos = [0., 0., 0.9, 1.0] # default pose
    env.step(pos)
    time.sleep(1)


    # print('current pose:', env.ur5.get_current_pose())

    for s in range(10000):
        pybullet.stepSimulation()
        time.sleep(0.02)

    print('test complete!')
