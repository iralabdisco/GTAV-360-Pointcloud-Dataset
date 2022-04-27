from math import degrees
import open3d as o3d
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as rot
import os

def tf_cam_to_velo():
    tr_cam_to_velo = np.zeros((3,3))
    tr_cam_to_velo[0,1] = -1
    tr_cam_to_velo[1,2] = -1
    tr_cam_to_velo[2,0] = 1
    tr_cam_to_velo = np.linalg.inv(tr_cam_to_velo)
    print(tr_cam_to_velo)
    return tr_cam_to_velo

def load_point_cloud_in_world_frame(frame: int, sector: int):
    print('loading', frame, 'pointcloud, sector', sector)
    pcd = from_bin_to_pcd('C:\\Users\\giovi\\Desktop\\Bicocca\\TESI_MAGISTRALE\\GTAVDataset\\object\\velodyne\\velodyne_{}\\{:06d}.bin'.format(sector, frame))
    T = load_transformation_matrix('C:\\Users\\giovi\\Desktop\\Bicocca\\TESI_MAGISTRALE\\GTAVDataset\\object\\pose\\pose_{}\\{:06d}.txt'.format(sector, frame))
    pcd.transform(T)
    #mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
    #mesh.translate(T[:3, 3])
    #o3d.visualization.draw_geometries([mesh, pcd])
    return pcd

def load_position(path_to_pose_file):
    pose_file = pd.read_csv(path_to_pose_file, header=None)
    t = np.float32(pose_file.iloc[2])    
    tmp = t[0]
    t[0] = t[1]
    t[1] = -tmp
    t[2] = -t[2]
    #print('world coordinates: ', t)
    return t

def load_rotation(path_to_pose_file):
    pose_file = pd.read_csv(path_to_pose_file, header=None)
    r = np.float32(pose_file.iloc[3])
    #print('world rotation: ', r)
    r = rot.from_euler('XYZ', r, degrees=True)
    return r.as_matrix()    

def load_transformation_matrix(path_to_pose_file):
    T = np.eye(4)
    T[:3, :3] = load_rotation(path_to_pose_file)
    T[:3, 3] = -load_position(path_to_pose_file)
    return T

def from_bin_to_pcd(path_to_bin_file):
    bin = np.fromfile(path_to_bin_file, dtype=np.float32)
    points = np.float32(bin.reshape((-1, 4))[:, 0:3])
    points = [p for p in points if np.abs(np.linalg.norm(p)) > 1]
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
    #print(t)
    points -= t
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    return pcd#.transform(T)

def load_360_point_cloud_global_location(frame, path):
    points = load_360_point_cloud(frame, path)
    t = load_360_global_location(frame, path)
    #tmp = t[0]
    t[0] = t[0]
    t[1] = -t[1]
    t[2] = t[2]
    #print(t)
    points += t
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    return pcd#.transform(T)

def load_360_point_cloud(frame, path):
    pcd = o3d.io.read_point_cloud(os.path.join(path, os.path.join('velodyne_360', '{:06d}.pcd'.format(frame))))
    return np.float32(pcd.points)

def load_360_point_cloud_o3d(frame, path):
    t = load_360_global_location(frame, path)
    pcd = o3d.io.read_point_cloud(os.path.join(path, os.path.join('velodyne_360', '{:06d}.pcd'.format(frame))))
    return pcd

def load_360_global_location(frame, path):
    print(os.path.join(path, os.path.join('location_360', '{:06d}.txt'.format(frame))))
    t = np.loadtxt(os.path.join(path, os.path.join('location_360', '{:06d}.txt'.format(frame))))
    #tmp = t[0]
    t[0] = t[0]
    t[1] = -t[1]
    t[2] = -t[2]
    return t

def load_point_cloud_location(frame, path, loc_type='relative'):
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
    t[1] = -t[1]
    t[2] = -t[2]
    return t