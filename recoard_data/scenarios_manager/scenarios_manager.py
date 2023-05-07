import os
from collections import OrderedDict
from yaml_utils import load_yaml
from scene_manager import ScenesManager

class ScenariosManager:
    def __init__(self, config_path ):
        scenario_params = load_yaml(os.path.join(config_path, "config.yaml"))
        self.scene_params = scenario_params
        root_dir = self.scene_params['root_dir']
        self.scenario_database = OrderedDict()
        output_dir = os.path.join( scenario_params['output_dir'] )
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

        self.scenarios = []
        configs_path = os.path.join(config_path, "lidar_config")
        # scenario_configs = [os.path.join(configs_path, f) 
        #                         for f in os.listdir(configs_path) if f.endswith('.yaml')]
        scenario_configs = [os.path.join(config_path, "scenario_config.yaml")]
        for i,lidar_config in enumerate(scenario_configs):
            path = os.path.join(output_dir, "sensors_%s"%i)
            if not os.path.exists(path):
                os.makedirs(path)
            curs = ScenesManager( lidar_config,
                                path
                            )
            self.scenarios.append(curs)
        
    def tick(self):
        for scenario in self.scenarios:
            scenario.start_simulator()
            run_flag = True
            while run_flag:
                run_flag = scenario.tick()
            scenario.close()


if __name__ == "__main__":
    config_path = "/data/carla_release/PythonAPI/recoard_data/hepes_yaml/"
    scenarios_manager = ScenariosManager(config_path)
    scenarios_manager.tick()
    print('recoard end')
