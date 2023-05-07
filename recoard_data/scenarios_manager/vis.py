import os
import numpy as np
import open3d as o3d

# 设置文件夹路径
# folder_path = '/data/opv2v/test/data_0/2'
# folder_path = '/data/opv2v/train/4_8/1'
folder_path = '/data/opv2v/test/data_0/2'

# 定义键盘事件回调函数
def key_callback(vis, key, scancode, action, mods):
    global current_file_index, current_pcd
    if key == ord('A') and action == 0:
        # 如果有前一个文件，加载前一个文件
        if current_file_index > 0:
            current_file_index -= 1
            current_pcd = o3d.io.read_point_cloud(pcd_files[current_file_index])
            vis.update_geometry(current_pcd)
            vis.poll_events()
            vis.update_renderer()
    elif key == ord('D') and action == 0:
        # 如果有后一个文件，加载后一个文件
        if current_file_index < len(pcd_files) - 1:
            current_file_index += 1
            current_pcd = o3d.io.read_point_cloud(pcd_files[current_file_index])
            vis.update_geometry(current_pcd)
            vis.poll_events()
            vis.update_renderer()


if __name__ == "__main__":
    # 获取文件夹中所有 PCD 文件
    pcd_files = [os.path.join(folder_path, f) for f in os.listdir(folder_path) if f.endswith('.pcd')]
    # 加载第一个 PCD 文件并可视化
    current_file_index = 0
    current_pcd = o3d.io.read_point_cloud(pcd_files[current_file_index])
    o3d.visualization.draw_geometries([current_pcd])
    # 创建可视化窗口并添加键盘事件回调函数
    o3d.visualization.draw_geometries_with_key_callbacks([current_pcd], key_callback)
