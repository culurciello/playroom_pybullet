# E. Culurciello
# March 2021

# multi-task desk environment
# based on: https://github.com/google-research/google-research/tree/master/playrooms

# desk object

import pybullet

DESK_URDF_PATH = "assets/desk/desk.urdf"

class Desk():
    def __init__(self ):
        self.desk = pybullet.loadURDF(DESK_URDF_PATH)
        self.num_joints = pybullet.getNumJoints(self.desk)
        self.joints = [pybullet.getJointInfo(self.desk, i) for i in range(self.num_joints)]
        self.control_joints = [j[0] for j in self.joints if j[2] == pybullet.JOINT_PRISMATIC]

        # print(self.num_joints)
        # print('DESK JOINTS:', self.joints)
        # print(self.control_joints)

        # print(self.joints[2]) # drawer handle
        # print(self.joints[4]) # slide handle
        # print(self.joints[5]) # button 1

