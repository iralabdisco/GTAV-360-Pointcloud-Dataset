import open3d as o3d
import numpy as np
import argparse, os, time
from tools import load_360_point_cloud_o3d, load_360_point_cloud

base_folder = os.environ['DEEPGTAV_EXPORT_DIR']


def options():
    parser = argparse.ArgumentParser(description='Create 360 from Velodyne Files')
    parser.add_argument('-r', '--run', required=False, type=str, default='object', help='run name (default: object)')
    parser.add_argument('--step', required=False, type=int, default=1)
    args = parser.parse_args()
    return args

def main():
    ARGS = options()
    dataset_folder = os.path.join(base_folder, ARGS.run)
    assert os.path.isdir(dataset_folder), '{} run does not exist!'.format(ARGS.run)

    frame_list = [int(x.split('.')[0]) for x in os.listdir(os.path.join(dataset_folder, 'velodyne_360'))]

    first = True

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name='GTAV 360 Sensor Data')
    render_option = vis.get_render_option()
    render_option.background_color = np.array([0.1529, 0.1569, 0.1333], np.float32)
    render_option.point_color_option = o3d.visualization.PointColorOption.ZCoordinate

    pcd = o3d.geometry.PointCloud()

    for frame in frame_list:
        print('computing frame {}'.format(frame))
        pcd.points = o3d.utility.Vector3dVector(load_360_point_cloud(frame, dataset_folder))
        if first:
            vis.add_geometry(pcd)
            first = False
        else:
            vis.update_geometry(pcd)
        ctr = vis.get_view_control()
        ctr.change_field_of_view(60.0)
        ctr.set_front([-0.09, -0.72, 0.68])
        ctr.set_lookat([1.54, -6.56, -17.05])
        ctr.set_up([-0.02, 0.68, 0.72])
        ctr.set_zoom(0.70)       
        vis.poll_events()
        vis.update_renderer()
        time.sleep(0.01)

    vis.destroy_window()

if __name__ == '__main__':
    main()