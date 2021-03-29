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

class DeskEnv:
    def __init__(self):
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 0)

        # assets:
        self.floor = pybullet.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), [0, 0, -1])
        self.desk = pybullet.loadURDF(DESK_URDF_PATH)

        self.num_joints = pybullet.getNumJoints(self.desk)
        print('Desk number of joints:', self.num_joints)
        self.control_joints = ["drawer_joint", "slide_joint", "button1_joint", "button2_joint", "button3_joint"]
        self.joint_type_list = ["REVOLUTE", "PRISMATIC", "SPHERICAL", "PLANAR", "FIXED"]
        self.joint_info = namedtuple("jointInfo", ["id", "name", "type", "lowerLimit", "upperLimit", "maxForce", "maxVelocity", "controllable"])
        self.joints = AttrDict()
        for i in range(self.num_joints):
            info = pybullet.getJointInfo(self.desk, i)
            jointID = info[0]
            jointName = info[1].decode("utf-8")
            jointType = self.joint_type_list[info[2]]
            jointLowerLimit = info[8]
            jointUpperLimit = info[9]
            jointMaxForce = info[10]
            jointMaxVelocity = info[11]
            controllable = True if jointName in self.control_joints else False
            info = self.joint_info(jointID, jointName, jointType, jointLowerLimit, jointUpperLimit, jointMaxForce, jointMaxVelocity, controllable)
            if info.type == "PRISMATIC":
                pybullet.setJointMotorControl2(self.desk, info.id, pybullet.POSITION_CONTROL, targetVelocity=0, force=0)
            self.joints[jointName] = info

        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 1)


    def set_joint_pos(self, joint_pos, repeat=100):
        poses = []
        indexes = []
        forces = []

        for i, name in enumerate(self.control_joints):
            joint = self.joints[name]
            poses.append(joint_pos[i])
            indexes.append(joint.id)
            forces.append(100)

        pybullet.setJointMotorControlArray(
            self.desk, indexes,
            pybullet.POSITION_CONTROL,
            targetPositions=joint_pos,
            forces=forces
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

    # operate [drawer, slide, button1, button2, button3]:
    joint_pos = [0.0, 0.0, 0.0, 0.0, 0.0]
    joint_limits = ((0.0, 0.4), (-0.1, 0.6), (-0.1, 0.0), (-0.1, 0.0), (-0.1, 0.0))
    for i in range(len(desk.control_joints)):
        joint_pos[i] = joint_limits[i][0]
        desk.set_joint_pos(joint_pos)
        # print(joint_limits[i][0])
        time.sleep(1)
        joint_pos[i] = joint_limits[i][1]
        desk.set_joint_pos(joint_pos)
        # print(joint_limits[i][1])
        time.sleep(1)
        joint_pos[i] = 0.0


    print('test complete!')
    