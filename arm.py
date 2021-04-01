# E. Culurciello
# March 2021

# multi-task desk environment
# based on: https://github.com/google-research/google-research/tree/master/playrooms

# arm / ur5 object

import math 
import pybullet
# import cameras
import numpy as np


ROBOT_URDF_PATH = "assets/ur5/ur5.urdf"


class UR5():
    def __init__(self,
                 pos=[0.55, 0.15, 0.6],
                 orientation=[0, 0, -1, 1],
                 camera_attached=False,
                 useIK=True, # here we use IK by default!
                 actionRepeat=1,
                 renders=False,
                 maxSteps=100,
                 simulatedGripper=True, # here suction cup simulated
                 ):

        self.pix_size = 0.003125
        self.obj_ids = {'fixed': [], 'rigid': [], 'deformable': []}
        self.rest_poses = [-math.pi, -math.pi/2, math.pi/2, -math.pi/2, -math.pi/2, 0]
        # self.agent_cams = cameras.RealSenseD415.CONFIG
        self.assets_root = 'assets/'
        self.simulatedGripper = simulatedGripper
        self.actionRepeat = actionRepeat
        self.useIK = useIK

        if self.useIK:
            self.action_dim = 4 # IK: 3 coordinates, 1 gripper
        else:
            self.action_dim = 7 # direct control: 6 arm DOF, 1 gripper
        
        # setup robot arm:
        flags = pybullet.URDF_USE_SELF_COLLISION
        self.arm = pybullet.loadURDF(ROBOT_URDF_PATH, pos, orientation, flags=flags)
        self.num_joints = pybullet.getNumJoints(self.arm)

        # end effector / gripper
        self.gripper_tip = 9  # Link ID of gripper

        # Get revolute joint indices of robot (skip fixed joints).
        self.joints = [pybullet.getJointInfo(self.arm, i) for i in range(self.num_joints)]
        self.control_joints = [j[0] for j in self.joints if j[2] == pybullet.JOINT_REVOLUTE]
        self.control_joints_gripper = [j[0] for j in self.joints if j[2] == pybullet.JOINT_PRISMATIC]


    def set_joint_angles(self, joint_angles):
        pybullet.setJointMotorControlArray(
            self.arm,
            jointIndices=self.control_joints,
            controlMode=pybullet.POSITION_CONTROL,
            targetPositions=joint_angles[0:6],
            targetVelocities=[0.01]*len(self.control_joints),
            positionGains=[0.5]*len(self.control_joints),
        )

    def movep(self, pose, speed=0.01):
        """Move UR5 to target end effector pose."""
        targj = self.solve_ik(pose)
        self.set_joint_angles(targj)
        # return self.movej(targj, speed)

    def solve_ik(self, pose):
        """Calculate joint configuration with inverse kinematics."""
        joints = pybullet.calculateInverseKinematics(
            bodyUniqueId=self.arm,
            endEffectorLinkIndex=self.gripper_tip,
            targetPosition=pose[0],
            targetOrientation=pose[1],
            lowerLimits=[-3 * np.pi / 2, -2.3562, -17, -17, -17, -17],
            upperLimits=[-np.pi / 2, 0, 17, 17, 17, 17],
            jointRanges=[np.pi, 2.3562, 34, 34, 34, 34],  # * 6,
            restPoses=np.float32(self.rest_poses).tolist(),
            maxNumIterations=100,
            residualThreshold=1e-5)
        joints = np.float32(joints)
        joints[2:] = (joints[2:] + np.pi) % (2 * np.pi) - np.pi
        return joints

    def _get_obs(self):
        # Get RGB-D camera image observations.
        obs = {'color': (), 'depth': ()}
        # for config in self.agent_cams:
          # color, depth, _ = self.render_camera(config)
          # obs['color'] += (color,)
          # obs['depth'] += (depth,)

        return obs

    def check_collisions(self):
        collisions = pybullet.getContactPoints()
        if len(collisions) > 0:
            # print("[Collision detected!] {}".format(datetime.now()))
            return True
        return False

    def get_current_pose(self):
        linkstate = pybullet.getLinkState(self.arm, self.gripper_tip, computeForwardKinematics=True)
        position, orientation = linkstate[0], linkstate[1]
        return (position, orientation)

    def get_joint_angles(self):
        j = pybullet.getJointStates(self.arm, [1,2,3,4,5,6])
        joints = [i[0] for i in j]
        return joints

    def gripper_open(self):
        gripper_pos = [0.0, 0.0] # open gripper position
        pybullet.setJointMotorControlArray(
            self.arm,
            jointIndices=self.control_joints_gripper,
            controlMode=pybullet.POSITION_CONTROL,
            targetPositions=gripper_pos,
            targetVelocities=[0.01]*len(self.control_joints_gripper),
            positionGains=[0.5]*len(self.control_joints_gripper),
        )


    def gripper_close(self):
        gripper_pos = [0.03, -0.03] # closed gripper position
        pybullet.setJointMotorControlArray(
            self.arm,
            jointIndices=self.control_joints_gripper,
            controlMode=pybullet.POSITION_CONTROL,
            targetPositions=gripper_pos,
            targetVelocities=[0.01]*len(self.control_joints_gripper),
            positionGains=[0.5]*len(self.control_joints_gripper),
        )


    def render_camera(self, config):
        """Render RGB-D image with specified camera configuration."""

        # OpenGL camera settings.
        lookdir = np.float32([0, 0, 1]).reshape(3, 1)
        updir = np.float32([0, -1, 0]).reshape(3, 1)
        rotation = pybullet.getMatrixFromQuaternion(config['rotation'])
        rotm = np.float32(rotation).reshape(3, 3)
        lookdir = (rotm @ lookdir).reshape(-1)
        updir = (rotm @ updir).reshape(-1)
        lookat = config['position'] + lookdir
        focal_len = config['intrinsics'][0]
        znear, zfar = config['zrange']
        viewm = pybullet.computeViewMatrix(config['position'], lookat, updir)
        fovh = (config['image_size'][0] / 2) / focal_len
        fovh = 180 * np.arctan(fovh) * 2 / np.pi

        # Notes: 1) FOV is vertical FOV 2) aspect must be float
        aspect_ratio = config['image_size'][1] / config['image_size'][0]
        projm = pybullet.computeProjectionMatrixFOV(fovh, aspect_ratio, znear, zfar)

        # Render with OpenGL camera settings.
        _, _, color, depth, segm = pybullet.getCameraImage(
            width=config['image_size'][1],
            height=config['image_size'][0],
            viewMatrix=viewm,
            projectionMatrix=projm,
            shadow=1,
            flags=pybullet.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX,
            renderer=pybullet.ER_BULLET_HARDWARE_OPENGL)

        # Get color image.
        color_image_size = (config['image_size'][0], config['image_size'][1], 4)
        color = np.array(color, dtype=np.uint8).reshape(color_image_size)
        color = color[:, :, :3]  # remove alpha channel
        if config['noise']:
          color = np.int32(color)
          color += np.int32(self._random.normal(0, 3, config['image_size']))
          color = np.uint8(np.clip(color, 0, 255))

        # Get depth image.
        depth_image_size = (config['image_size'][0], config['image_size'][1])
        zbuffer = np.array(depth).reshape(depth_image_size)
        depth = (zfar + znear - (2. * zbuffer - 1.) * (zfar - znear))
        depth = (2. * znear * zfar) / depth
        if config['noise']:
          depth += self._random.normal(0, 0.003, depth_image_size)

        # Get segmentation image.
        segm = np.uint8(segm).reshape(depth_image_size)

        return color, depth, segm