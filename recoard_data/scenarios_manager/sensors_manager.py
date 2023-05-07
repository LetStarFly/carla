import importlib
import os
from collections import OrderedDict
from semantic_lidar import SemanticLidar

class SensorManager:
    def __init__(self, agent_id, world, config_yaml, vehicle=None):
        self.agent_id = agent_id
        self.world = world
        self.sensor_list = []
        self.sensor_meta = OrderedDict()

        for sensor_id, sensor_content in config_yaml.items():
            sensor = None
            sensor_name = sensor_content['name']
            if 'lidar' in sensor_name:
                sensor = SemanticLidar

            senosr_instance = sensor(str(sensor_id), 
                                    self.world, 
                                    sensor_content['args'])
            
            self.sensor_list.append(senosr_instance)

    def run_step(self, cur_timestamp, output_dir):  # 运行
        # world_snapshot = self.world.get_snapshot()
        # cur_timestamp = world_snapshot.timestamp
        for sensor_instance in self.sensor_list:
            sensor_name = sensor_instance.name
            sensor_instance.visualize_data()
            
            meta_info = sensor_instance.tick()  # 在激光雷达中 用于获取车辆的id
            self.sensor_meta.update({sensor_name: meta_info})
            output_folder = os.path.join(output_dir,
                                         sensor_instance.agent_id)

            if not os.path.exists(output_folder):
                os.makedirs(output_folder)

            sensor_instance.data_dump(output_folder,
                                      cur_timestamp)

    
    def destroy(self):
        for sensor_instance in self.sensor_list:
            sensor_instance.destroy()