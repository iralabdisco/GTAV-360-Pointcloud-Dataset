from turtle import pd
import open3d as o3d
import numpy as np
import pandas as pd
import os

def from_bin_to_pcd(path_to_bin_file):
    bin = np.fromfile(path_to_bin_file, dtype=np.float32)
    points = np.float32(bin.reshape((-1, 4))[:, 0:3])
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    return pcd

def from_bin_to_points(path_to_bin_file):
    bin = np.fromfile(path_to_bin_file, dtype=np.float32)
    points = np.float32(bin.reshape((-1, 4))[:, 0:3])
    return points

def load_360_point_cloud_relative_location(frame, path):
    points = load_360_point_cloud(frame, path)
    t = load_point_cloud_location(frame, path, 'relative')
    points += t
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    return pcd#.transform(T)

def load_360_point_cloud_global_location(frame, path):
    points = load_360_point_cloud(frame, path)
    t = load_point_cloud_location(frame, path, 'global')
    points += t
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    return pcd#.transform(T)

def load_360_point_cloud(frame, path):
    pcd = o3d.io.read_point_cloud(os.path.join(path, os.path.join('velodyne_360', '{:06d}.pcd'.format(frame))))
    return np.float32(pcd.points)

def load_point_cloud_location(frame, path, loc_type='relative'):
    T = np.eye(4)
    location_file = pd.read_csv(os.path.join(path, os.path.join('location_360', '{:06d}.txt'.format(frame))), header=None)
    if loc_type == 'relative':
        t = np.float32(location_file.iloc[1])
    elif loc_type == 'global':
        t = np.float32(location_file.iloc[0])
    elif loc_type == 'step':
        t = np.float32(location_file.iloc[2])
    else:
        print('wrong type!')
    t[0] = -t[0]
    t[1] = t[1]
    t[2] = t[2]
    return t