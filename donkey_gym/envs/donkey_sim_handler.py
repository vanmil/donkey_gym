'''
file: donkey_sim.py
author: Tawn Kramer
date: 2018-08-31
'''

import base64
import time
from io import BytesIO
import math

import numpy as np
from PIL import Image
from io import BytesIO
import base64


from donkey_gym.core.tcp_server import IMesgHandler


class DonkeyUnitySimHandler(IMesgHandler):

    def __init__(self, level, verbose=False, cam_resolution=None):
        self.iSceneToLoad = level
        self.sock = None
        self.loaded = False
        self.verbose = verbose
        
        # sensor size - height, width, depth
        self.camera_img_size = cam_resolution
        self.image_array = np.zeros(self.camera_img_size)
        
        self.hit = "none"
        self.cte = 0.0 # cross-track-error
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.speed = 0.0
        self.scene_names = []
        self.current_scene = None
        self.fns = {'telemetry' : self.on_telemetry,
                    "scene_selection_ready" : self.on_scene_selection_ready,
                    "scene_names": self.on_recv_scene_names,
                    "car_loaded" : self.on_car_loaded }


    def reset_variables(self):
        if self.verbose:
            print("reseting")
        self.image_array = np.zeros(self.camera_img_size)
        self.hit = "none"
        self.cte = 0.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.speed = 0.0

    def get_sensor_size(self):
        return self.camera_img_size

    def observe_image(self):
        return self.image_array

    def observe_position(self):
        return (self.x, self.y, self.z)

    def observe_cte(self):
        return self.cte

    def observe_speed(self):
        return self.speed

    def observe_hit(self):
        return self.hit


    ## ------ Socket interface ----------- ##
    # Callbacks
    def on_connect(self, socketHandler): # gets called from SimHandler init
        self.sock = socketHandler

    def on_disconnect(self):
        self.sock = None

    def on_recv_message(self, message):
        if not 'msg_type' in message:
            print('expected msg_type field')
            return

        msg_type = message['msg_type']
        if msg_type in self.fns:
            self.fns[msg_type](message)
        else:
            print('unknown message type', msg_type)

    def on_telemetry(self, data):
        imgString = data["image"]
        image = Image.open(BytesIO(base64.b64decode(imgString)))

        #always update the image_array as the observation loop will hang if not changing.
        self.image_array = np.asarray(image)

        self.x = data["pos_x"]
        self.y = data["pos_y"]
        self.z = data["pos_z"]
        self.speed = data["speed"]

        #Cross track error not always present.
        #Will be missing if path is not setup in the given scene.
        #It should be setup in the 4 scenes available now.
        if "cte" in data:
            self.cte = data["cte"]
        
        self.hit = data["hit"]

    def on_scene_selection_ready(self, data):
        if self.verbose:
            print("scene selection ready")
        self.send_get_scene_names()

    def on_car_loaded(self, data):
        if self.verbose:
            print("car loaded")
        self.loaded = True

    def on_recv_scene_names(self, data):
        if data:
            self.scene_names = data['scene_names']
            if self.verbose:
                print("SceneNames:", self.scene_names)
            self.send_load_scene(self.scene_names[self.iSceneToLoad])

    # Senders

    def send_control(self, steer, throttle, brake):
        if not self.loaded:
            return
        msg = { 'msg_type' : 'control', 'steering': steer.__str__(), 'throttle':throttle.__str__(), 'brake': brake.__str__() }
        self.queue_message(msg)

    def send_reset_car(self):
        msg = { 'msg_type' : 'reset_car' }
        self.queue_message(msg)

    def send_get_scene_names(self):
        msg = { 'msg_type' : 'get_scene_names' }
        self.queue_message(msg)

    def send_load_scene(self, scene_name):
        print("Trying to load scene")
        msg = { 'msg_type' : 'load_scene', 'scene_name' : scene_name }
        self.queue_message(msg)
        self.current_scene = scene_name

    def send_exit_scene(self):
        msg = {'msg_type': 'exit_scene'}
        self.queue_message(msg)

    def send_regen_road(self, road_style, rand_seed, turn_increment):
        msg = { 'msg_type' : 'regen_road', 'road_style':road_style.__str__(), 'rand_seed':rand_seed.__str__(), 'turn_increment': turn_increment.__str__()}
        self.queue_message(msg)

    def queue_message(self, msg):
        if self.sock is None:
            if self.verbose:
                print('skiping:', msg)
            return

        if self.verbose:
            print('sending', msg)
        self.sock.queue_message(msg)
