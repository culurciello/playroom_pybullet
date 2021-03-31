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
        print('DESK JOINTS:', self.joints)
        # print('\n\n')
        # print(self.joints[2]) # drawer handle
        # print('\n\n')
        # print(self.joints[4]) # slide handle
        # print('\n\n')
        # print(self.joints[5]) # button 1

        print(self.control_joints)


    # def movep(self, pose, speed=0.01):
    #     """Move UR5 to target end effector pose."""
    #     targj = self.solve_ik(pose)
    #     self.set_joint_angles(targj)
    #     # return self.movej(targj, speed)

    # def solve_ik(self, pose):
    #     """Calculate joint configuration with inverse kinematics."""
    #     joints = pybullet.calculateInverseKinematics(
    #         bodyUniqueId=self.arm,
    #         endEffectorLinkIndex=self.gripper_tip,
    #         targetPosition=pose[0],
    #         targetOrientation=pose[1],
    #         lowerLimits=[-3 * np.pi / 2, -2.3562, -17, -17, -17, -17],
    #         upperLimits=[-np.pi / 2, 0, 17, 17, 17, 17],
    #         jointRanges=[np.pi, 2.3562, 34, 34, 34, 34],  # * 6,
    #         restPoses=np.float32(self.rest_poses).tolist(),
    #         maxNumIterations=100,
    #         residualThreshold=1e-5)
    #     joints = np.float32(joints)
    #     joints[2:] = (joints[2:] + np.pi) % (2 * np.pi) - np.pi
    #     return joints