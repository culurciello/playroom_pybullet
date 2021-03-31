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
# import numpy as np

DESK_URDF_PATH = "assets/desk/desk.urdf"
PLANE_URDF_PATH = os.path.join(pybullet_data.getDataPath(), "plane.urdf")


class DeskEnv:
    def __init__(self):
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 0)

        # assets:
        self.floor = pybullet.loadURDF(PLANE_URDF_PATH, [0, 0, -0.001])
        self.desk = pybullet.loadURDF(DESK_URDF_PATH, [0.0, 0.0, 0.2])

        self.num_joints = pybullet.getNumJoints(self.desk)
        self.joints = [pybullet.getJointInfo(self.desk, i) for i in range(self.num_joints)]
        self.control_joints = [j[0] for j in self.joints if j[2] == pybullet.JOINT_PRISMATIC]

        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 1)




    def set_joint_pos(self, idx, joint_pos, repeat=100, velocity=0, force=100):
        pybullet.setJointMotorControl2(
            self.desk,
            jointIndex=idx,
            controlMode=pybullet.POSITION_CONTROL,
            targetPosition=joint_pos,
            targetVelocity=velocity,
            force=force
        )
        for s in range(repeat):
            pybullet.stepSimulation()


# testing usage:
if __name__ == '__main__':
    # start pybullet and GUI:
    client = pybullet.connect(pybullet.GUI)
    pybullet.setTimeStep(1.0/240.0)
    pybullet.setGravity(0,0,-9.8)
    pybullet.setRealTimeSimulation(False)
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI,0)

    pybullet.resetDebugVisualizerCamera(
                cameraDistance=0.5,
                cameraYaw=180,
                cameraPitch=-35,
                cameraTargetPosition=[0,0.5,0.85])
        
    
    # load and initialize items:
    desk = DeskEnv()

    print(desk.joints)
    print('Testing joints:', desk.control_joints)
    # operate [drawer, slide, button1, button2, button3]:
    joint_pos = [0.0, 0.0, 0.0, 0.0, 0.0]
    joint_limits = ((0.0, 0.4), (-0.1, 0.6), (0.0, -0.1), (0.0, -0.1), (0.0, -0.1))
    for i in range(len(desk.control_joints)):
        idx = desk.control_joints[i]
        print('Moving joint:', idx)
        pos = joint_limits[i][1]
        # print('pos', pos)
        desk.set_joint_pos(idx, pos)
        time.sleep(1)
        pos = joint_limits[i][0]
        # print('pos', pos)
        desk.set_joint_pos(idx, pos)
        time.sleep(1)

    print('test complete!')
    