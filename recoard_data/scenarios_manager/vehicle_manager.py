import os
import random
import sys
from collections import OrderedDict

import carla
import numpy as np
import logging
from carla import VehicleLightState as vls
from blue_pre import BLUE_PRINT_LIB

def get_actor_blueprints(world, filter ="vehicle.*",  generation='all'):
    bps = world.get_blueprint_library().filter(filter)

    bps = [x for x in bps if x.id in BLUE_PRINT_LIB.keys()]
    bps = [x for x in bps if int(x.get_attribute('number_of_wheels')) == 4]
    bps = [x for x in bps if not x.id.endswith('microlino')]
    bps = [x for x in bps if not x.id.endswith('carlacola')]
    bps = [x for x in bps if not x.id.endswith('cybertruck')]
    bps = [x for x in bps if not x.id.endswith('t2')]
    bps = [x for x in bps if not x.id.endswith('sprinter')]
    bps = [x for x in bps if not x.id.endswith('firetruck')]
    bps = [x for x in bps if not x.id.endswith('ambulance')]

    # If the filter returns only one bp, we assume that this one needed
    # and therefore, we ignore the generation
    if len(bps) == 1:
        return bps

    if generation.lower() == "all":
        return bps

    try:
        int_generation = int(generation)
        # Check if generation is in available generations
        if int_generation in [1, 2]:
            bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
            return bps
        else:
            print("   Warning! Actor Generation is not valid. No actor will be spawned.")
            return []
    except:
        print("   Warning! Actor Generation is not valid. No actor will be spawned.")
        return []


class VehicleManager:
    # 管理车辆的生成和销毁
    

    def __init__(self, client, world, nums):
        self.client = client
        self.world = world
        self.number_of_vehicles = nums

        # model = 'vehicle.lincoln.mkz_2017'
        # # retrive the blueprint library
        # blueprint_library = self.world.get_blueprint_library()
        # cav_bp = blueprint_library.find(model)
        # # cav is always green
        # color = '0, 0, 255'
        # cav_bp.set_attribute('color', color)
        
        self.traffic_manager = self.client.get_trafficmanager(8000)
        self.traffic_manager.set_global_distance_to_leading_vehicle(200)


        spawn_points = self.world.get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)

        if self.number_of_vehicles < number_of_spawn_points:
            self.number_of_vehicles >=  number_of_spawn_points

        random.shuffle(spawn_points)

        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        FutureActor = carla.command.FutureActor

        # --------------
        # Spawn vehicles
        # --------------
        bps = get_actor_blueprints(self.world)
        self.vehicles_list = []
        batch=[]
        for n, transform in enumerate(spawn_points):
            if n >= self.number_of_vehicles:
                break
            cav_bp = random.choice(bps)
            if cav_bp.has_attribute('base_type'):
                print(cav_bp.get_attribute('base_type'))
            if cav_bp.has_attribute('color'):
                color = random.choice(cav_bp.get_attribute('color').recommended_values)
                cav_bp.set_attribute('color', color)
            batch.append(SpawnActor(cav_bp, transform)
                .then(SetAutopilot(FutureActor, True, self.traffic_manager.get_port()))
                )
        
        for response in self.client.apply_batch_sync(batch, True):
            if response.error:
                logging.error(response.error)
            else:
                self.vehicles_list.append(response.actor_id)

        # Example of how to use Traffic Manager parameters
        self.traffic_manager.global_percentage_speed_difference(30.0)


    def destroy(self):
        self.client.apply_batch([carla.command.DestroyActor(x) for x in self.vehicles_list])
