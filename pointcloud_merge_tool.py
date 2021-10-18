import numpy as np
import copy
import open3d as o3d
import rosbag
import ros_numpy
import tqdm


def load_from_bag(bag_file, source_topic, target_topic):
    bag = rosbag.Bag(bag_file)
    source_points = []
    target_points = []
    for topic, msg, t in tqdm.tqdm(bag.read_messages()):
        if topic in [source_topic, target_topic]:
            cloud_array = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
            x = cloud_array["x"].reshape(-1)
            y = cloud_array["y"].reshape(-1)
            z = cloud_array["z"].reshape(-1)
            if topic == source_topic:
                source_points.append(np.c_[x, y, z])
            else:
                target_points.append(np.c_[x, y, z])

    source_pc = o3d.geometry.PointCloud()
    source_pc.points = o3d.utility.Vector3dVector(np.concatenate(source_points))
    source_pc = source_pc.voxel_down_sample(0.1)
    target_pc = o3d.geometry.PointCloud()
    target_pc.points = o3d.utility.Vector3dVector(np.concatenate(target_points))
    target_pc = target_pc.voxel_down_sample(0.1)
    return source_pc, target_pc


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])


def pick_points(pcd):
    print("")
    print(
        "1) Please pick at least three correspondences using [shift + left click]"
    )
    print("   Press [shift + right click] to undo point picking")
    print("2) After picking points, press 'Q' to close the window")
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()  # user picks points
    vis.destroy_window()
    print("")
    return vis.get_picked_points()


def demo_manual_registration(source, target):
    print("Demo for manual ICP")
    print("Visualization of two point clouds before manual alignment")
    draw_registration_result(source, target, np.identity(4))

    # pick points from two point clouds and builds correspondences
    picked_id_source = pick_points(source)
    picked_id_target = pick_points(target)
    assert (len(picked_id_source) >= 3 and len(picked_id_target) >= 3)
    assert (len(picked_id_source) == len(picked_id_target))
    corr = np.zeros((len(picked_id_source), 2))
    corr[:, 0] = picked_id_source
    corr[:, 1] = picked_id_target

    # estimate rough transformation using correspondences
    print("Compute a rough transform using the correspondences given by user")
    p2p = o3d.pipelines.registration.TransformationEstimationPointToPoint()
    trans_init = p2p.compute_transformation(source, target,
                                            o3d.utility.Vector2iVector(corr))

    # point-to-point ICP for refinement
    print("Perform point-to-point ICP refinement")
    threshold = 0.03  # 3cm distance threshold
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint())
    draw_registration_result(source, target, reg_p2p.transformation)
    print("")


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Pointcloud merge tool.")
    parser.add_argument("bag_file", type=str, help="Input bag file.")
    parser.add_argument("--source", type=str, default="/pointcloud/source", help="Source pointcloud topic name.")
    parser.add_argument("--target", type=str, default="/pointcloud/target", help="Target pointcloud topic name.")
    args = parser.parse_args()
    source, target = load_from_bag(args.bag_file, args.source, args.target)
    demo_manual_registration(source, target)