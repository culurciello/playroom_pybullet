# E. Culurciello
# March 2021

# multi-task desk environment
# based on: https://github.com/google-research/google-research/tree/master/playrooms

import os, math, random
import time

import numpy as np

from gym import spaces
import gym

import pybullet
import pybullet_data

from arm import UR5
from desk import Desk


PLANE_URDF_PATH = os.path.join(pybullet_data.getDataPath(), "plane.urdf")
CUBE_URDF_PATH = os.path.join(pybullet_data.getDataPath(), "cube_small.urdf")


# x,y,z distance
def goal_distance(goal_a, goal_b):
    assert goal_a.shape == goal_b.shape
    return np.linalg.norm(goal_a - goal_b, axis=-1)


# x,y distance
def goal_distance2d(goal_a, goal_b):
    assert goal_a.shape == goal_b.shape
    return np.linalg.norm(goal_a[0:2] - goal_b[0:2], axis=-1)


class DeskGymEnv(gym.Env):
    def __init__(self,
                 camera_attached=False,
                 useIK=True, # uses IK by default!
                 actionRepeat=50,
                 renders=False,
                 maxSteps=100,
                 simulatedGripper=True, # 2-finger gripper
                 randObjPos=False,
                 task=0, # target number
                 learning_param=0,
                 sampling_freq = 240.0):

        self.name = 'ur5GymEnv'
        self.sampling_freq = sampling_freq # simulation sampling frequency
        self.pix_size = 0.003125
        self.obj_ids = {'fixed': [], 'rigid': [], 'deformable': []}
        self.rest_poses = [-math.pi, -math.pi/2, math.pi/2, -math.pi/2, -math.pi/2, 0]
        # self.agent_cams = cameras.RealSenseD415.CONFIG
        self.assets_root = 'assets/'
        self.simulatedGripper = simulatedGripper
        self.renders = renders
        self.actionRepeat = actionRepeat
        self.useIK = useIK
        self.maxSteps = maxSteps
        self.randObjPos = randObjPos
        self.task = task
        self.learning_param = learning_param
        self.offset = 0.15 # random object pos max offsets
        self.desired_pick_height = 0.2 # desired height to reach for picking up objects

        # setup pybullet sim:
        if self.renders:
            pybullet.connect(pybullet.GUI)
            target = pybullet.getDebugVisualizerCamera()[11]
            pybullet.resetDebugVisualizerCamera(
                cameraDistance=0.5,
                cameraYaw=180,
                cameraPitch=-35,
                cameraTargetPosition=[0,0.5,0.85])
        else:
            pybullet.connect(pybullet.DIRECT)

        pybullet.setTimeStep(1./self.sampling_freq)
        pybullet.setGravity(0,0,-9.8)
        # pybullet.setRealTimeSimulation(False)
        pybullet.setRealTimeSimulation(True)
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 0)
        # self.gox = pybullet.addUserDebugParameter("orientationX",-3.14, 3.14, 0)
        # self.goy = pybullet.addUserDebugParameter("orientationY",-3.14, 3.14, 0)
        # self.goz = pybullet.addUserDebugParameter("orientationZ",-3.14, 3.14, 0)
        # self.gow = pybullet.addUserDebugParameter("orientationW",-3.14, 3.14, 1)
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0)
        pybullet.setPhysicsEngineParameter(enableFileCaching=0)
        # pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_WIREFRAME,1)

        # setup scene / desk:
        self.desk = Desk()

        # setup robot arm:
        self.plane = pybullet.loadURDF(PLANE_URDF_PATH, [0, 0, -0.001])
        self.ur5 = UR5()

        # object:
        self.initial_obj_pos = [0.2, -0.5, 0.6] # initial object pos
        self.initial_target_pos = [0.4, -0.1, 0.0] # initial drop-off position
        self.obj = pybullet.loadURDF(CUBE_URDF_PATH, self.initial_obj_pos)
        self.obj_ids['rigid'].append(self.obj)

        # gym info:
        color_tuple = [
            gym.spaces.Box(0, 255, config['image_size'] + (3,), dtype=np.uint8)
            for config in self.ur5.agent_cams
        ]
        depth_tuple = [
            gym.spaces.Box(0.0, 20.0, config['image_size'], dtype=np.float32)
            for config in self.ur5.agent_cams
        ]
        self.observation_space = gym.spaces.Dict({
            'color': gym.spaces.Tuple(color_tuple),
            'depth': gym.spaces.Tuple(depth_tuple),
        })
        self._action_bound = 1.0 # delta limits
        action_high = np.array([self._action_bound] * self.ur5.action_dim)
        self.action_space = spaces.Box(-action_high, action_high, dtype='float32')

        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 1)

        self.reset()


    def reset(self):
        self.stepCounter = 0
        self.current_task = 0
        self.terminated = False
        self.ur5_or = [0.0, 1/2*math.pi, 0.0]
        self.target_pos = self.initial_target_pos
        obj_pos = self.initial_obj_pos

        # pybullet.addUserDebugText('X', self.obj_pos, [0,1,0], 1) # display goal
        if self.randObjPos:
            obj_pos = np.array(self.initial_obj_pos) + [random.random()*self.offset, random.random()*self.offset, 0.0]
            self.target_pos = np.array(self.target_pos) + [random.random()*self.offset, random.random()*self.offset, 0.0]

        pybullet.resetBasePositionAndOrientation(self.obj, obj_pos, [0.,0.,0.,1.0]) # reset object pos

        # reset robot simulation and position:
        self.ur5.set_joint_angles(self.rest_poses)

        # Reset end effector.
        self.ur5.gripper_open()

        # step simulator for 1 s:
        for i in range(int(self.sampling_freq)):
            pybullet.stepSimulation()

        # get obs and return:
        self.getExtendedObservation()
        self.tool_initial_pos = self.tool_pos
        return self.observation
    
    
    def step(self, action):
        action = np.array(action)
        arm_action = action[0:self.ur5.action_dim-1].astype(float) # X, Y, Z - range: [-1,1]
        gripper_action = action[self.ur5.action_dim-1].astype(float) # gripper - range: [-1=closed,1=open]

        # get current position:
        cur_p = self.ur5.get_current_pose()
        
        # actuate:
        # print(cur_p[1])
        # self.ur5.movep( (arm_action, np.asarray(cur_p[1])) )
        # self.ur5.movep( (arm_action, np.asarray([0.,0.3,0.,0.9])) )

        if self.task==0: 
            orientation = np.asarray([0.0, 1.0, 0.0, 0.0])
        elif self.task==1:
            orientation = np.asarray([0.7, 0.0, 0.0, 1.0])
            # orientation = np.asarray(
            # [pybullet.readUserDebugParameter(self.gox),
            #  pybullet.readUserDebugParameter(self.goy),
            #  pybullet.readUserDebugParameter(self.goz),
            #  pybullet.readUserDebugParameter(self.gow)] )

        self.ur5.movep( (arm_action, orientation))
        
        if gripper_action >= 0.0: 
            self.ur5.gripper_close()
        elif gripper_action < 0.0:
            self.ur5.gripper_open()
        
        # step simulator:
        for i in range(self.actionRepeat):
            pybullet.stepSimulation()
            if self.renders: time.sleep(1./self.sampling_freq)
        
        self.getExtendedObservation()
        reward = self.compute_reward(self.achieved_goal, self.desired_goal, 0) # call this after getting obs!
        done = self.my_task_done()

        info = {}

        self.stepCounter += 1

        return self.observation, reward, done, info


    # observations are: arm (tip/tool) position, arm acceleration, ...
    def getExtendedObservation(self):
        # sensor values:
        js = self.ur5.get_joint_angles()
        self.tool_pos,_ = self.ur5.get_current_pose() # XYZ, no angles
        self.obj_pos,_ = pybullet.getBasePositionAndOrientation(self.obj)
        # self.observation = np.array(np.concatenate((self.tool_pos, self.obj_pos, js)))

        # we define tasks as: 0-reach obj, 1-lift ojb, 2-move to target, 3-drop obj
        self.goal_pos = self.obj_pos
        if self.current_task == 2: # reach target pos
            self.goal_pos = self.target_pos
            if self.renders: pybullet.addUserDebugText('X', self.goal_pos, [0,1,0], 1) # display goal

        self.achieved_goal = np.array(self.tool_pos)
        self.desired_goal = np.array(self.goal_pos)

        self.observation = self.ur5._get_obs()


    def my_task_done(self):
        # NOTE: need to call compute_reward before this to check termination!
        c = (self.current_task == self.task+1 or self.stepCounter > self.maxSteps)
        return c


    # hardcoded reward for pick/place task
    def compute_reward(self, achieved_goal, desired_goal, info):
        reward = 0

        self.obj_dist = goal_distance(np.array(self.tool_pos), 
                                        np.array(self.obj_pos))
        self.target_dist = goal_distance(np.array(self.tool_pos), 
                                        np.array(self.target_pos))

        reward = -self.obj_dist * 10 -self.target_dist * 10

        # task 0: reach object:
        if self.current_task == 0 and self.obj_dist < self.learning_param:
            # print('Successful object reach')
            reward += 1
            self.current_task += 1
        
        # task 1: lift up:
        if self.current_task == 1 and self.obj_pos[2] > self.desired_pick_height:
            # print('Successful picked up!')
            reward += 1
            self.current_task += 1

        # task 2: reach target: needs more care to place
        if self.current_task == 2 and self.target_dist < 2*self.learning_param:
            # print('Successful target reach')
            reward += 1
            self.current_task += 1
        
        # task 3: lift up:
        if self.current_task == 3 and self.tool_pos[2] > self.desired_pick_height:
            # print('Successful drop!')
            reward += 1
            self.current_task += 1

        # check collisions:
        if self.ur5.check_collisions(): 
            reward += -1
            # print('Collision!')

        return reward