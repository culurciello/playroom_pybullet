# E. Culurciello
# March 2021

# test for desk environment
# based on: https://github.com/google-research/google-research/tree/master/playrooms

import os, sys 
import time
import pybullet
# import pybullet_data
# from collections import namedtuple
# from attrdict import AttrDict
import numpy as np
from gym_env import DeskGymEnv
from argparse import ArgumentParser

title = 'PyBullet UR5 robot and desk'

def get_args():
    parser = ArgumentParser(description=title)
    arg = parser.add_argument
    arg('--task', type=int, default=0, help='task to test: 0-open drawer, 1-open slide, 2-push button1, 10-test')
    args = parser.parse_args()
    return args

args = get_args() # all input arguments


# testing usage:
if __name__ == '__main__':
    # initialize env:
    env = DeskGymEnv(renders=True, actionRepeat=50)

    # pos is: [x, y, z, gripper] gripper > 0 is closed, < 0 is open

    # task: 0-open drawer, 1-open slide, 2-push button1, 
    # 10-test

    env.task = args.task

    if args.task==0:

        pos = [0.1, 0.0, 0.45, -1.0] # reach drawer + open gripper
        obs, reward, done, info = env.step(pos)

        pos = [0.1, 0.2, 0.50, -1.0] # open drawer + open gripper
        obs, reward, done, info = env.step(pos)

    elif args.task==1:
        # pos = [-0.4, -0.4, 0.6, -1.0] # reach slide + open gripper
        pos = [0., 0., 0.9, -1.0]
        obs, reward, done, info = env.step(pos)

        # pos = [-0.4, -0.4, 0.6, -1.0] # reach slide + open gripper
        # env.step(pos)

    elif args.task==2:

        pos = [0.25, -0.35, 0.7, 0.0] # reach buttons
        env.step(pos)

    elif args.task==10:

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

    else:
        print('Select a usable task!')
        error(1)

    # just wait for a while:
    for s in range(10000):
        pybullet.stepSimulation()
        time.sleep(0.02)

    # print('current pose:', env.ur5.get_current_pose())

    print('test complete!')
