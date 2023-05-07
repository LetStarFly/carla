"""
This is mainly used to filter out objects that is not in the sight
of cameras.
"""
import weakref

import carla
import cv2
import numpy as np
from base_sensor import BaseSensor
import os
import open3d as o3d
from yaml_utils import save_yaml_wo_overwriting
from collections import OrderedDict

class SemanticLidar(BaseSensor):
    # lidar_count, world, config
    def __init__(self, agent_id, world, config, vehicle=None):
        super().__init__(agent_id, vehicle, world, config )
        
        if vehicle is not None:
            world = vehicle.get_world()
            relative_position = config['relative_pose']
            spawn_position = self.spawn_point_estimation(relative_position,
                                                  vehicle['pose'])
        else:
            spawn_position = config['spawn_position']
        
        self.world = world
        self.agent_id = agent_id
        # 获取Lidar，并修改设置
        blueprint = self.world.get_blueprint_library(). \
            find('sensor.lidar.ray_cast_semantic')

        # set attribute based on the configuration
        blueprint.set_attribute('upper_fov', str(config['upper_fov']))
        blueprint.set_attribute('lower_fov', str(config['lower_fov']))
        blueprint.set_attribute('channels', str(config['channels']))
        blueprint.set_attribute('range', str(config['range']))
        blueprint.set_attribute(
            'points_per_second', str(
                config['points_per_second']))
        blueprint.set_attribute(
            'rotation_frequency', str(
                config['rotation_frequency']))

        self.name = "semantic_lidar" + str(agent_id)
        self.thresh = config['thresh']

        carla_location = carla.Location(
                            x = spawn_position[0],
                            y = spawn_position[1],
                            z = spawn_position[2] )
        carla_rotation = carla.Rotation(
                            roll = spawn_position[3],
                            yaw = spawn_position[4],
                            pitch = spawn_position[5]
                            )
        spawn_point = carla.Transform(carla_location, carla_rotation)

        if vehicle is not None:
            self.sensor = self.world.spawn_actor(
                blueprint, spawn_point, attach_to=vehicle)
        else:
            self.sensor = self.world.spawn_actor(blueprint, spawn_point)

        # lidar data
        self.points = None
        self.obj_idx = None
        self.obj_tag = None

        self.timestamp = None
        self.frame = 0

        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda event: SemanticLidar._on_data_event(
                weak_self, event))

    @staticmethod
    def _on_data_event(weak_self, event):
        """Semantic Lidar  method"""
        self = weak_self()
        if not self:
            return

        # shape:(n, 6)
        data = np.frombuffer(event.raw_data, dtype=np.dtype([
            ('x', np.float32), ('y', np.float32), ('z', np.float32),
            ('CosAngle', np.float32), ('ObjIdx', np.uint32),
            ('ObjTag', np.uint32)]))
        intensity = np.zeros_like(data['x'])
        print(intensity.shape)
        # (x, y, z, intensity)
        self.points = np.array([data['x'], data['y'], data['z'], intensity]).T
        self.obj_tag = np.array(data['ObjTag'])
        self.obj_idx = np.array(data['ObjIdx'])

        self.data = data
        self.frame = event.frame
        self.timestamp = event.timestamp


    @staticmethod
    def spawn_point_estimation(relative_position, global_position):

        pitch = 0
        carla_location = carla.Location(x=0, y=0, z=0)

        if global_position is not None:
            carla_location = carla.Location(
                x=global_position[0],
                y=global_position[1],
                z=global_position[2])
            pitch = -35

        if relative_position == 'front':
            carla_location = carla.Location(x=carla_location.x + 2.5,
                                            y=carla_location.y,
                                            z=carla_location.z + 1.0)
            yaw = 0

        elif relative_position == 'right':
            carla_location = carla.Location(x=carla_location.x + 0.0,
                                            y=carla_location.y + 0.3,
                                            z=carla_location.z + 1.8)
            yaw = 100

        elif relative_position == 'left':
            carla_location = carla.Location(x=carla_location.x + 0.0,
                                            y=carla_location.y - 0.3,
                                            z=carla_location.z + 1.8)
            yaw = -100
        else:
            carla_location = carla.Location(x=carla_location.x - 2.0,
                                            y=carla_location.y,
                                            z=carla_location.z + 1.5)
            yaw = 180

        carla_rotation = carla.Rotation(roll=0, yaw=yaw, pitch=pitch)
        spawn_point = carla.Transform(carla_location, carla_rotation)

        return spawn_point

    def tick(self):
        while self.obj_idx is None or self.obj_tag is None or \
                self.obj_idx.shape[0] != self.obj_tag.shape[0]:
            continue

        # label 10 is the vehicle
        vehicle_idx = self.obj_idx[self.obj_tag == 10]
        # each individual instance id
        vehicle_unique_id = list(np.unique(vehicle_idx))
        vehicle_id_filter = []

        # 获取对应的车辆的id
        for veh_id in vehicle_unique_id:
            if vehicle_idx[vehicle_idx == veh_id].shape[0] > self.thresh:
                vehicle_id_filter.append(veh_id)
        
        self.vehicle_id_filter = vehicle_id_filter
        # these are the ids that are visible
        return vehicle_id_filter


    def data_dump(self, output_root, cur_timestamp):
        cur_timestamp_str = '{:0>6d}'.format(cur_timestamp)
        # 保存点云文件
        output_path_name = os.path.join(output_root,
                                       cur_timestamp_str + '.pcd')
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.points[:,:3])
        pcd.colors = o3d.utility.Vector3dVector(self.points[:, 3:].repeat(3, axis=1))
        o3d.io.write_point_cloud(output_path_name, pcd)
        
        # 保存id_pose
        save_yaml_name = os.path.join(output_root, 
                                      cur_timestamp_str + '.yaml')
        position = self.sensor.get_transform()
        cords = [position.location.x,
            position.location.y,
            position.location.z,
            position.rotation.roll,
            position.rotation.yaw,
            position.rotation.pitch]
        
        # 保存对应的vechile_pose
        actors = self.world.get_actors().filter('vehicle.*')
        actors_info = OrderedDict()
        for actor in actors:
            if actor.id in self.vehicle_id_filter:

                # assuming 'actor' is the CARLA Actor you want to get the relative position for
                actor_transform = actor.get_transform()
                actor_bounding_box = actor.bounding_box

                # get the location of the bounding box center relative to the actor's transform

                center = [0,0,0.7]

                # center = bounding_box.center
                extent = actor_bounding_box.extent
                extent = [extent.x, extent.y, extent.z]

                transform = actor.get_transform()
                location = transform.location
                location = [location.x, location.y, location.z]

                rotation = transform.rotation
                rotation = [rotation.roll, rotation.yaw, rotation.pitch]
                velocity  = actor.get_velocity()
                speed = np.linalg.norm([velocity.x, velocity.y, velocity.z])
                
                actors_info[actor.id ] = {
                    "angle": rotation,
                    "center": center,
                    "extent": extent,
                    "location": location,
                    "speed": speed
                }
        lidar_info = {'lidar_pose': cords,
                      'vehicls': actors_info}
        
        save_yaml_wo_overwriting(lidar_info,
                            save_yaml_name)