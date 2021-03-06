#!/usr/bin/env python
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml

run_pick_place = False

pcl_objects_pub = None
object_markers_pub = None
detected_objects_pub = None
rotation_pub = None


def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster


# Create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"] = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict


def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)


def outlier_removal(cloud):
    outlier_filter = cloud.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(50)
    # Any point with a mean distance larger than
    # (mean distance + 1. * std_dev) will be considered an outlier
    outlier_filter.set_std_dev_mul_thresh(.7)
    return outlier_filter.filter()


def voxel_grid_filter(cloud):
    vox = cloud.make_voxel_grid_filter()
    vox.set_leaf_size(.01, .01, .01)
    return vox.filter()


def pass_through_filter(cloud):
    passthrough = cloud.make_passthrough_filter()
    passthrough.set_filter_field_name('z')
    passthrough.set_filter_limits(.6, 1.1)
    cloud = passthrough.filter()

    passthrough = cloud.make_passthrough_filter()
    passthrough.set_filter_field_name('x')
    passthrough.set_filter_limits(.4, 1.1)
    return passthrough.filter()


def ransac(cloud):
    segmenter = cloud.make_segmenter()
    segmenter.set_model_type(pcl.SACMODEL_PLANE)
    segmenter.set_method_type(pcl.SAC_RANSAC)
    segmenter.set_distance_threshold(.02)

    inliers, coefficients = segmenter.segment()
    return inliers, coefficients


def euclidean_clustering(point_cloud):
    tree = point_cloud.make_kdtree()
    ec = point_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.05)
    ec.set_MinClusterSize(50)
    ec.set_MaxClusterSize(2500)
    ec.set_SearchMethod(tree)
    return ec.Extract()


def world_number(objects):
    names = [x['name'] for x in objects]
    if 'sticky_notes' in names:
        return 3
    if 'book' in names:
        return 2
    return 1


def pcl_callback(pcl_msg):
    cloud = ros_to_pcl(pcl_msg)
    cloud = outlier_removal(cloud)
    cloud = voxel_grid_filter(cloud)
    cloud = pass_through_filter(cloud)
    objects_message = pcl_to_ros(cloud)

    table_inliers, coefficients = ransac(cloud)
    table_cloud = cloud.extract(table_inliers)
    objects_cloud = cloud.extract(table_inliers, negative=True)
    point_cloud = XYZRGB_to_XYZ(objects_cloud)
    clusters = euclidean_clustering(point_cloud)

    table_message = pcl_to_ros(table_cloud)
    pcl_table_pub.publish(table_message)

    cluster_color = get_color_list(len(clusters))
    color_cluster_point_list = []
    for j, indices in enumerate(clusters):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([
                point_cloud[indice][0],
                point_cloud[indice][1],
                point_cloud[indice][2],
                rgb_to_float(cluster_color[j]),
            ])

    # Create a new cloud with an unique color per cluster
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    objects_message = pcl_to_ros(cluster_cloud)
    pcl_objects_pub.publish(objects_message)

    detected_objects_labels = []
    detected_objects = []
    for index, pts_list in enumerate(clusters):
        pcl_cluster = objects_cloud.extract(pts_list)
        ros_cluster = pcl_to_ros(pcl_cluster)

        # Extract histogram features
        color_histogram = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((color_histogram, nhists))

        # Prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1, -1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(point_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label, label_pos, index))

        detected_object = DetectedObject()
        detected_object.label = label
        detected_object.cloud = ros_cluster
        detected_objects.append(detected_object)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))
    detected_objects_pub.publish(detected_objects)

    try:
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass


has_collision_map = False


def pr2_mover(detected_objects):
    global has_collision_map
    labels = []
    test_scene_num = Int32()
    arm_name = String()
    object_name = String()
    place_pose = Pose()
    pick_pose = Pose()
    yaml_output = []

    object_list_param = rospy.get_param('/object_list')
    dropboxes = rospy.get_param('/dropbox')

    if run_pick_place:
        if not has_collision_map:
            has_collision_map = True
            for angle in [3.14 / 2, 0, -3.14 / 2, 0]:
                rotation_pub.publish(angle)
                rospy.sleep(8)

    world = world_number(object_list_param)
    test_scene_num.data = world
    for i, obj in enumerate(detected_objects):
        object_name.data = str(obj.label)
        object_group = next(x['group'] for x in object_list_param if x['name'] == obj.label)
        labels.append(obj.label)
        points_arr = ros_to_pcl(obj.cloud).to_array()
        centroid = np.mean(points_arr, axis=0)[:3]

        pick_pose.position.x, pick_pose.position.y, pick_pose.position.z = (
            np.asscalar(x) for x in centroid)
        group_info = {
            'red': {
                'arm': 'left',
                'place_position': dropboxes[0]['position'],
            },
            'green': {
                'arm': 'right',
                'place_position': dropboxes[1]['position'],
            },
        }
        group = group_info[object_group]
        arm_name.data = group['arm']
        place_pose.position.x, place_pose.position.y, place_pose.position.z = group['place_position']

        yaml_output.append(make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose))

        if run_pick_place:
            rospy.wait_for_service('pick_place_routine')
            try:
                pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)
                resp = pick_place_routine(test_scene_num, object_name, arm_name, pick_pose, place_pose)
                print("Response: ", resp.success)
            except rospy.ServiceException as e:
                print("Service call failed: {}".format(e))

    print('YAML output: {}'.format(yaml_output))
    send_to_yaml('output.yaml', yaml_output)


if __name__ == '__main__':
    rospy.init_node('object_recognition', anonymous=True)
    pcl_sub = rospy.Subscriber("/pr2/world/points", PointCloud2, pcl_callback, queue_size=1)
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)
    rotation_pub = rospy.Publisher('/pr2/world_joint_controller/command', Float64, queue_size=1)

    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    print('Spinning...')
    rospy.spin()
    print('Done!')
