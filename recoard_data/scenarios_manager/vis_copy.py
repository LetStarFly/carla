import os
import numpy as np
import open3d as o3d

# 设置文件夹路径
# folder_path = '/data/opv2v/test/data_0/2'
# folder_path = '/data/opv2v/train/4_8/1'
folder_path = '/data/data/output3/data_0/semantic_lidar/lidar2/point'

if __name__ == "__main__":
    # 获取文件夹中所有 PCD 文件
    pcd_files = [os.path.join(folder_path, f) for f in os.listdir(folder_path) if f.endswith('.ply')]
    # 加载第一个 PCD 文件并可视化
    current_file_index = 0
    current_pcd = o3d.io.read_point_cloud(pcd_files[current_file_index])
    o3d.visualization.draw_geometries([current_pcd])

