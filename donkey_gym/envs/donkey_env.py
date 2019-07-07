'''
file: donkey_env.py
author: Tawn Kramer
date: 2018-08-31
'''
import os
from threading import Thread
import random
import time

import numpy as np
import gym
from gym import error, spaces, utils
from gym.utils import seeding
from donkey_gym.envs.donkey_sim_controller import DonkeyUnitySimContoller
from donkey_gym.envs.donkey_proc import DonkeyUnityProcess
from donkey_gym.envs.donkey_ex import SimFailed

class DonkeyEnv(gym.Env):
    """
    OpenAI Gym Environment for Donkey
    """

    metadata = {
        "render.modes": ["human", "rgb_array"],
    }

    ACTION_NAMES = ["steer", "throttle"]
    STEER_LIMIT_LEFT = -1.0
    STEER_LIMIT_RIGHT = 1.0
    THROTTLE_MIN = 0.0
    THROTTLE_MAX = 5.0
    VAL_PER_PIXEL = 255

    def __init__(self, level, frame_skip=2):

        print("starting DonkeyGym env")
        
        # start Unity simulation subprocess
        self.proc = DonkeyUnityProcess()
        
        try:
            exe_path = os.environ['DONKEY_SIM_PATH']
        except:
            print("Missing DONKEY_SIM_PATH environment var. you must start sim manually")
            exe_path = "self_start"

        try:
            port_offset = 0
            #if more than one sim running on same machine set DONKEY_SIM_MULTI = 1
            random_port = os.environ['DONKEY_SIM_MULTI']=='1'
            if random_port:
                port_offset = random.randint(0, 1000)
        except:
            pass
        
        try:
            port = int(os.environ['DONKEY_SIM_PORT']) + port_offset 
        except:
            port = 9091 + port_offset
            print("Missing DONKEY_SIM_PORT environment var. Using default:", port)
            
        try:
            headless = os.environ['DONKEY_SIM_HEADLESS']=='1'
        except:
            print("Missing DONKEY_SIM_HEADLESS environment var. Using defaults")
            headless = False

        self.proc.start(exe_path, headless=headless, port=port)

        # start simulation com
        self.simulator = DonkeyUnitySimContoller(level=level, port=port)
        
        # steering and throttle
        self.action_space = spaces.Box(low=np.array([self.STEER_LIMIT_LEFT, self.THROTTLE_MIN]),
            high=np.array([self.STEER_LIMIT_RIGHT, self.THROTTLE_MAX]), dtype=np.float32 )

        # camera sensor data
        self.observation_space = spaces.Box(0, self.VAL_PER_PIXEL, self.simulator.get_sensor_resolution(), dtype=np.uint8)

        # simulation related variables.
        self.seed()

        # Frame Skipping
        self.frame_skip = frame_skip

        # wait until loaded
        self.simulator.wait_until_loaded()

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        for i in range(self.frame_skip):
            self.simulator.take_action(action)
            observation, reward, done, info = self.simulator.observe()
        return observation, reward, done, info

    def reset(self, reset_scene_to=None):
        self.simulator.reset(reset_scene_to)
        observation, reward, done, info = self.simulator.observe()
        time.sleep(1)
        return observation

    def render(self, mode="human", close=False):
        pass

    # def regen_road(self, road_style, rand_seed, turn_increment):
    #     self.simulator.regen_road(road_style, rand_seed, turn_increment)

    def __del__(self):
        self.close()

    def close(self):
        self.proc.quit()



## ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ##

class GeneratedRoadsEnv(DonkeyEnv):

    def __init__(self):
        super(GeneratedRoadsEnv, self).__init__(level=0)

class WarehouseEnv(DonkeyEnv):

    def __init__(self):
        super(WarehouseEnv, self).__init__(level=1)

class AvcSparkfunEnv(DonkeyEnv):

    def __init__(self):
        super(AvcSparkfunEnv, self).__init__(level=2)

class GeneratedTrackEnv(DonkeyEnv):

    def __init__(self):
        super(GeneratedTrackEnv, self).__init__(level=3)
