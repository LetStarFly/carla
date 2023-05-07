import os
import random
import sys
from collections import OrderedDict

import carla
import numpy as np
from yaml_utils import load_yaml
from sensors_manager import SensorManager
from vehicle_manager import VehicleManager

class ScenesManager:
    def __init__(self, config_path, output_dir):
        self.scenes_config = load_yaml(config_path)
        self.output_root = output_dir
        

    def start_simulator(self):
        """
        Connect to the carla simulator for recoard dataset.
        """
        simulation_config = self.scenes_config['world']
        fixed_delta_seconds = simulation_config['fixed_delta_seconds']

        # setup the carla client
        self.client = \
            carla.Client('127.0.0.1', simulation_config['client_port'])
        self.client.set_timeout(10.0)  
        self.world = self.client.load_world(simulation_config['map'])
        # self.world = self.client.get_world()

        # setup the new setting
        self.origin_settings = self.world.get_settings()
        new_settings = self.world.get_settings()
        new_settings.synchronous_mode = True
        new_settings.fixed_delta_seconds = fixed_delta_seconds
        self.world.apply_settings(new_settings)
        
        # get map
        self.carla_map = self.world.get_map()
        # spectator
        self.spectator = self.world.get_spectator()
        location = carla.Location(30, -85, 100)
        rotation = carla.Rotation(pitch =-90)
        self.spectator.set_transform(carla.Transform(location, rotation))
        self.cur_count = 0
        self.cur_dataset = 0

        sensors_config = self.scenes_config['sensors_list']
        self.sensors_manage = SensorManager(-1,
                              self.world,
                              sensors_config
                                )  # output_roor
        
        self.vehicle_manage = VehicleManager(
                            self.client,
                            self.world,
                            simulation_config['vechile_num']
        )

    def tick(self):
        self.cur_count += 1
        if self.cur_count % 600 == 0:
            self.cur_dataset += 1
        output_folder = os.path.join(self.output_root, "data_%s" %self.cur_dataset)
        if not os.path.exists(output_folder):
                os.makedirs(output_folder)
        self.world.tick()
        self.sensors_manage.run_step(self.cur_count, output_folder)
        if self.cur_dataset > 100:
            return False
        return True

    def sensor_destory(self):
        self.sensors_manage.destroy()


    def vehicle_destory(self):
        self.vehicle_manage.destroy()
        
    def close(self):
        self.world.apply_settings(self.origin_settings)
        self.vehicle_destory()
        self.sensor_destory()
        actor_list = self.world.get_actors()
        for actor in actor_list:
            actor.destroy()