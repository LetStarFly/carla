import os
import numpy as np
import open3d as o3d

# 设置文件夹路径
# folder_path = '/data/opv2v/test/data_0/2'
# folder_path = '/data/opv2v/train/4_8/1'
folder_path = '/data/data/output3/data_0/semantic_lidar/lidar1/point/100.015494.ply'

if __name__ == "__main__":
    # 获取文件夹中所有 PCD 文件
    # 加载第一个 PCD 文件并可视化

    current_pcd = o3d.io.read_point_cloud(folder_path)
    o3d.visualization.draw_geometries([current_pcd])

