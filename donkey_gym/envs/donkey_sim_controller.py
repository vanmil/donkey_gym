'''
file: donkey_sim.py
author: Tawn Kramer
date: 2018-08-31
'''

from threading import Thread
import asyncore
import time
import math

from donkey_gym.core.tcp_server import SimServer
from donkey_gym.envs.donkey_ex import SimFailed
from donkey_gym.envs.donkey_sim_handler import DonkeyUnitySimHandler
from donkey_gym.core.fps import FPSTimer


class DonkeyUnitySimContoller():

    def __init__(self, level, port=9090, max_cte=5.0, verbose=False, cam_resolution=(120, 160, 3)):
        
        self.max_cte = max_cte
        self.last_obs = None
        self.timer = FPSTimer()
        self.cam_resolution = cam_resolution # the resolution that is used by Unity, at the moment this is 120x160
        self.address = ('0.0.0.0', port)

        self.handler = DonkeyUnitySimHandler(level, verbose=verbose, cam_resolution=cam_resolution)

        try:
            self.server = SimServer(self.address, self.handler)
        except OSError:
            print('raising custom error')
            raise SimFailed("failed to listen on address %s" % self.address)

        self.thread = Thread(target=asyncore.loop)
        self.thread.daemon = True
        self.thread.start()


    def wait_until_loaded(self):
        print("Checking if simulator is loaded")
        while not self.handler.loaded:
            print("waiting for sim handler to be loaded (for scene to start)..")
            time.sleep(3.0)

    def regen_road(self, road_style="0", rand_seed="10", turn_increment="1"):
        self.handler.send_regen_road(road_style, rand_seed, turn_increment)

    def reset(self, reset_scene_to=None):
        self.last_obs = None
        self.handler.reset_variables()

        if reset_scene_to is not None:
            self.handler.iSceneToLoad = reset_scene_to
            self.handler.send_exit_scene()
        else:
            self.reset_car()

    def get_sensor_resolution(self):
        return self.cam_resolution

    def take_action(self, action):
        self.handler.send_control(action[0], action[1], 0.0)

    def observe(self):
        while self.last_obs is self.handler.observe_image():
            time.sleep(1.0 / 120.0)

        obs = self.last_obs = self.handler.observe_image()
        pos = self.handler.observe_position()
        cte = self.handler.observe_cte()
        speed = self.handler.observe_speed()
        hit = self.handler.observe_hit()
        done = self._determine_episode_over(cte, hit)
        reward = self._calc_reward(cte, speed, done, hit)
        info = {'pos': pos, 'cte' : cte, "speed": speed, "hit": hit }
        return obs, reward, done, info


    def observe_seperate(self):
        while self.last_obs is self.handler.observe_image():
            time.sleep(1.0 / 120.0)

        obs = self.last_obs = self.handler.observe_image()
        pos = self.handler.observe_position()
        cte = self.handler.observe_cte()
        speed = self.handler.observe_speed()
        hit = self.handler.observe_hit()

        return = {'obs': obs, 'pos': pos, 'cte' : cte, "speed": speed, "hit": hit }


    def _calc_reward(self, cte, speed, done, hit):
        if done:
            return -1.0
        if cte > self.max_cte:
            return -1.0
        if hit != "none":
            return -2.0
        return 1.0 - (cte / self.max_cte) * speed    #going fast close to the center of lane yeilds best reward


    def _determine_episode_over(self, cte, hit):
        if math.fabs(cte) > 2 * self.max_cte:
            return False
        elif math.fabs(cte) > self.max_cte:
            return True
        elif hit != "none":
            return True

    def reset_car(self):
        self.handler.send_reset_car()
        self.timer.reset()
        time.sleep(1)