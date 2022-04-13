import pandas as pd
import open3d as o3d
import numpy as np
from tools import from_bin_to_points
from itertools import repeat
import argparse, os, multiprocessing, csv

base_folder = os.environ['DEEPGTAV_EXPORT_DIR']


def options():
    parser = argparse.ArgumentParser(description='Create 360 from Velodyne Files')
    parser.add_argument('-r', '--run', required=False, type=str, default='object', help='run name (default: object)')
    args = parser.parse_args()
    return args

def create_360(frame, velodyne_folder, pcd_360_folder):
    print('computing frame {}'.format(frame))
    try:
        first = True
        points_list = []
        for sector in range(1, 5):
            velopath = os.path.join(velodyne_folder, os.path.join('velodyne_{:01d}'.format(sector), '{:06d}.bin'.format(frame)))
            points = from_bin_to_points(velopath)
            if first:
                points_list = np.float32(points)
                first = False
            else:
                points_list = np.vstack((points_list, np.float32(points)))
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_list)
        #o3d.visualization.draw_geometries([pcd]) # for debug
        o3d.io.write_point_cloud(os.path.join(pcd_360_folder, '{:06d}.pcd'.format(frame)), pcd, write_ascii=True)
    except:
        print('error! \n')

def main(ARGS):
    dataset_folder = os.path.join(base_folder, ARGS.run)
    assert os.path.isdir(dataset_folder), '{} run does not exist!'.format(ARGS.run)
    cpu_count = multiprocessing.cpu_count()
    vehicle_pose_file = pd.read_csv(dataset_folder + '/vehicle_pose.txt', header=None, names=['x', 'y', 'z', 'roll', 'pitch', 'yaw'])
    n_frames = len(vehicle_pose_file.index)
    velodyne_folder = os.path.join(dataset_folder ,'velodyne')

    pcd_360_folder = os.path.join(dataset_folder ,'velodyne_360')
    if not os.path.isdir(pcd_360_folder):
        os.mkdir(pcd_360_folder)

    pose_360_folder = os.path.join(dataset_folder ,'location_360')
    if not os.path.isdir(pose_360_folder):
        os.mkdir(pose_360_folder)


    #with multiprocessing.Pool(cpu_count) as pool:
    #    check = pool.starmap(create_360, zip(range(0, n_frames), repeat(velodyne_folder), repeat(pcd_360_folder)))
    
    pose_prec = np.float32(vehicle_pose_file.iloc[0])
    prec_location = pose_prec[0:3]
    start_location = prec_location
    for frame in range(0, n_frames):
        pose = np.float32(vehicle_pose_file.iloc[frame])
        global_location = pose[0:3]
        relative_traslation = start_location - global_location
        step_traslation = prec_location - global_location
        prec_location = global_location
        with open(os.path.join(pose_360_folder, '{:06d}.txt'.format(frame)), 'w', newline='') as f:
            write = csv.writer(f)
            write.writerow(global_location)
            write.writerow(relative_traslation)
            write.writerow(step_traslation)

    print('\npointclouds created! \n')

if __name__ == '__main__':
    ARGS = options()
    main(ARGS)