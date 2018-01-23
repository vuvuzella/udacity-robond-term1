#!/usr/bin/env python

# Import modules
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


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    # Convert ROS msg to PCL data
    pcl_data = ros_to_pcl(pcl_msg)
    
    # Statistical outlier filtering
    # Much like the previous filters,
    # we start by creating a filter object: 
    outlier_filter = pcl_data.make_statistical_outlier_filter()
    mean = 7   # mean number of neighboring points
    thresh = 0.3 # Set threshold scale factor
    # Set the number of neighboring points to analyze for any given point
    outlier_filter.set_mean_k(mean)
    # Any point with a mean distance larger than global
    # (mean distance+x*std_dev) will be considered outlier
    outlier_filter.set_std_dev_mul_thresh(thresh)
    # filtered point cloud data
    cloud_filtered = outlier_filter.filter()

    # Voxel Grid Downsampling
    # TODO: comments
    LEAF_SIZE = 0.008
    vox = cloud_filtered.make_voxel_grid_filter()
    # vox = pcl_data.make_voxel_grid_filter()
    # leaf size for x, y and z dimensions
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud_filtered = vox.filter()   # produce a downsampled point cloud data

    # PassThrough Filter
    # Set z-axis pass through filter
    filter_axis = 'z'    # set the axis to 'pass through'
    axis_min = 0.6       # minimum z
    axis_max = 0.85      # max z
    passthrough = cloud_filtered.make_passthrough_filter()
    passthrough.set_filter_field_name(filter_axis)
    passthrough.set_filter_limits(axis_min, axis_max)
    cloud_filtered = passthrough.filter()
    # Set x-axis pass through filter
    # to Filter out the drop boxes being mistakenly clustered
    filter_x_axis = 'x'    # set the axis to 'pass through'
    axis_x_min = 0.4
    axis_x_max = 2
    passthrough = cloud_filtered.make_passthrough_filter()
    passthrough.set_filter_field_name(filter_x_axis)
    passthrough.set_filter_limits(axis_x_min, axis_x_max)
    cloud_filtered = passthrough.filter()

    # RANSAC Plane Segmentation
    # TODO: Comments
    max_distance = 0.009
    ransac = cloud_filtered.make_segmenter()
    ransac.set_model_type(pcl.SACMODEL_PLANE)
    ransac.set_method_type(pcl.SAC_RANSAC)
    ransac.set_distance_threshold(max_distance)

    # Extract inliers and outliers
    # TODO: Comments
    inliers, coefficients = ransac.segment()
    cloud_table = cloud_filtered.extract(inliers, negative=False)
    cloud_objects = cloud_filtered.extract(inliers, negative=True)

    # Euclidean Clustering
    # convert object point cloud data to xyz for segmentation
    white_cloud = XYZRGB_to_XYZ(cloud_objects)  
    kd_tree = white_cloud.make_kdtree()

    # Create Cluster-Mask Point Cloud to visualize each cluster separately
    tolerance = 0.03
    min_cluster_size = 150
    max_cluster_size = 1100
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(tolerance)
    ec.set_MinClusterSize(min_cluster_size)
    ec.set_MaxClusterSize(max_cluster_size)
    # Search the k-dimension tree for clusters according to set parameters
    ec.set_SearchMethod(kd_tree)
    cluster_indices = ec.Extract()  # Get the cluster groups
    # Visualization of the Eucledian Segmentation Step
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, index in enumerate(indices):
            color_cluster_point_list.append([
                white_cloud[index][0],  # Red
                white_cloud[index][1],  # Green
                white_cloud[index][2],   # Blue
                rgb_to_float(cluster_color[j])
            ])

    # new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # Convert PCL data to ROS messages
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cloud_cluster = pcl_to_ros(cluster_cloud)

    # Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cloud_cluster)

# Exercise-3:

    # Array containers to get the pint cloud data and the labels
    detected_objects_labels = []
    detected_objects = []

    # Classify the clusters! (loop through each detected cluster one at a time)

    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster 
        # from the extracted outliers (cloud_objects)
        pcl_cluster = cloud_objects.extract(pts_list)
        sample_cloud = pcl_to_ros(pcl_cluster)

        # Compute the associated feature vector
        # Using hsv instead of RGB
        chists = compute_color_histograms(sample_cloud, using_hsv=True)
        normals = get_normals(sample_cloud)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))

        # Make the prediction
        # retrieve the label for the result and add
        # it to the detected_objects_labels list
        prediction = clf.predict(scaler.transform(feature.reshape(1, -1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += 0.3 # TODO: What is this? offset?
        object_markers_pub.publish(make_label(label, label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = sample_cloud
        detected_objects.append(do)

        # Do some logging
        rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))


    # Publish the list of detected objects
    print 'Publishing detected objects'
    detected_objects_pub.publish(detected_objects)

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        # Add code to handle when no objects are detected..
        pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # Initialize variables
    object_name = String()
    object_group = ''
    centroid = None
    point_cloud = None
    arm_name = String()
    test_scene_num = Int32()
    test_scene_num.data = 3 # TODO: can I get this automatically?
    pick_pose = Pose()
    place_pose = Pose()
    dropbox_right_pos = ''
    dropbox_left_pos = ''
    dict_list = []
    obj_detected = False

    # Get/Read parameters
    object_list_param = rospy.get_param('/object_list')


    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # Loop through the pick list
    for item in object_list_param:
        # Parse parameters into individual variables
        object_name.data = item['name']
        object_group = item['group']

        # Get the PointCloud for a given object and obtain it's centroid
        for obj in object_list:
            if obj.label == object_name.data:
                point_cloud = obj.cloud
                obj_detected = True
                break
            else:
                pass

        # If object in pick list is not part of objects detected,
        # skip to the next object in pick list
        if obj_detected != True:
            continue
        else:
            obj_detected = False

        points_arr = ros_to_pcl(point_cloud).to_array()
        centroid = np.mean(points_arr, axis=0)[:3]
        pick_pose.position.x = np.asscalar(centroid[0])
        pick_pose.position.y = np.asscalar(centroid[1])
        pick_pose.position.z = np.asscalar(centroid[2])

        # Create 'place_pose' for the object
        dropbox_param = rospy.get_param('/dropbox')
        for dest in dropbox_param:
            if dest['name'] == 'left':
                dropbox_left_pos = dest['position']
            elif dest['name'] == 'right':
                dropbox_right_pos = dest['position']
            else:
                # add more place destinations
                pass
        if object_group == 'green':
            place_pose.position.x = float(dropbox_right_pos[0])
            place_pose.position.y = float(dropbox_right_pos[1])
            place_pose.position.z = float(dropbox_right_pos[2])
        else:
            place_pose.position.x = float(dropbox_left_pos[0])
            place_pose.position.y = float(dropbox_left_pos[1])
            place_pose.position.z = float(dropbox_left_pos[2])

        # Assign the arm to be used for pick_place
        if object_group == 'green':
            arm_name.data = 'right'
        else:
            arm_name.data = 'left'

        # Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
        yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
        dict_list.append(yaml_dict)

        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')

        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # Insert your message variables to be sent as a service request
            # resp = pick_place_routine(test_scene_num, object_name, arm_name, pick_pose, place_pose)

            # print ("Response: ",resp.success)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    # Output your request parameters into output yaml file
    send_to_yaml("test_world_" + str(test_scene_num.data) + ".yaml", dict_list)

if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node('3d_perception', anonymous=True)

    # Create Subscribers
    # Subscribe to /pr2/world/points
    # data type: PointCloud2, callback: pcl_callback, queue size: 1
    rospy.Subscriber('/pr2/world/points', pc2.PointCloud2, pcl_callback, queue_size=1)


    # Create Publishers
    pcl_objects_pub = rospy.Publisher('/pcl_objects', PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher('/pcl_table', PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher('/pcl_cluster', PointCloud2, queue_size=1)
    object_markers_pub = rospy.Publisher('/object_markers', Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher('/detected_objects', DetectedObjectsArray, queue_size=1)

    # Load Model From disk
    model = pickle.load(open('./model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()