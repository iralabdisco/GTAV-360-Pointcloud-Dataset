import open3d as o3d
import numpy as np
import argparse, os
from tools import load_360_point_cloud_o3d, load_360_point_cloud_global_location

base_folder = os.environ['DEEPGTAV_EXPORT_DIR']


def options():
    parser = argparse.ArgumentParser(description='visualize map')
    parser.add_argument('-r', '--run', required=False, type=str, default='object', help='run name (default: object)')
    args = parser.parse_args()
    return args

if __name__ == '__main__':
    ARGS = options()
    dataset_folder = os.path.join(base_folder, ARGS.run)
    assert os.path.isdir(dataset_folder), '{} run does not exist!'.format(ARGS.run)

    velo_360_folder = os.path.join(dataset_folder, 'velodyne_360')
    location_360_folder = os.path.join(dataset_folder, 'location_360')

    frames = [int(x.split('.')[0]) for x in os.listdir(velo_360_folder)]

    point_map = []
    first = True
    for frame in frames:
        pcd = load_360_point_cloud_global_location(frame, dataset_folder)
        pcd = pcd.voxel_down_sample(voxel_size=1)
        if first:
            point_map = np.float32(pcd.points)
            first = False
        else:
            point_map = np.vstack((point_map, np.float32(pcd.points)))
    
    map_pcd = o3d.geometry.PointCloud()
    map_pcd.points = o3d.utility.Vector3dVector(point_map)
    map_pcd = map_pcd.voxel_down_sample(voxel_size=1)
    map_pcd.paint_uniform_color([0, 0.706, 1])
    o3d.io.write_point_cloud(os.path.join(dataset_folder, '{}_complete_map.pcd'.format(ARGS.run)), map_pcd, write_ascii=True)