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

def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:
    # Convert ROS msg to PCL data
    pcl_data = ros_to_pcl(pcl_msg)

    # Steps:
    # 1. Downsample your point cloud by applying the Voxel Grid Filter
    # 2. Apply a Pass Through Filter to isolate the table and objects
    # 3. Perform RANSAC plane fitting to identify the table
    # 4. Use the ExtractIndices Filter to create new point clouds containg
    #    the table and objects separately (called cloud_table and cloud_objects)

    # Voxel Grid Downsampling
    LEAF_SIZE = 0.01
    vox = pcl_data.make_voxel_grid_filter()
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud_filtered = vox.filter()

    # PassThrough Filter
    filter_axis = 'z'
    axis_min = 0.759
    axis_max = 1.1
    passthrough = cloud_filtered.make_passthrough_filter()
    passthrough.set_filter_field_name(filter_axis)
    passthrough.set_filter_limits(axis_min, axis_max)
    cloud_filtered = passthrough.filter()

    # RANSAC Plane Segmentation
    max_distance = 0.01
    ransac = cloud_filtered.make_segmenter()
    ransac.set_model_type(pcl.SACMODEL_PLANE)
    ransac.set_method_type(pcl.SAC_RANSAC)
    ransac.set_distance_threshold(max_distance)

    # Extract inliers and outliers
    inliers, coefficients = ransac.segment()
    cloud_table = cloud_filtered.extract(inliers, negative=False)
    cloud_objects = cloud_filtered.extract(inliers, negative=True)

    # Euclidean Clustering
    # white_cloud would be a spatial information array data type
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    kd_tree = white_cloud.make_kdtree()

    # Create Cluster-Mask Point Cloud to visualize each cluster separately
    # Cluster extraction
    ec = white_cloud.make_EuclideanClusterExtraction()

    # Set tolerances for distance threshold 
    # as well as minimum and maximum cluster size (in points)
    # NOTE: These are poor choices of clustering parameters
    # Your task is to experiment and find values that work for segmenting objects.
    ec.set_ClusterTolerance(0.05)
    ec.set_MinClusterSize(200)
    ec.set_MaxClusterSize(2000)

    # Search the k-d tree for clusters
    ec.set_SearchMethod(kd_tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()
    # print cluster_indices

    # Visualization of Euclidean Segmentation step
    # Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, index in enumerate(indices):
            color_cluster_point_list.append([
                white_cloud[index][0],
                white_cloud[index][1],
                white_cloud[index][2],
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

# Exercise-3 TODOs: 

    # Array containers to get the pint cloud data and the labels
    detected_objects_labels = []
    detected_objects = []

    # Classify the clusters! (loop through each detected cluster one at a time)
    # cycle through each segmented clusters
    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster from the extracted outliers (cloud_objects)
        pcl_cluster = cloud_objects.extract(pts_list)
        # convert the cluster from pcl to ROS using helper function
        ros_cluster = pcl_to_ros(pcl_cluster)
        sample_cloud = ros_cluster
        sample_cloud_arr = ros_to_pcl(sample_cloud).to_array()

        # Compute the associated feature vector
        # Extract histogram features
        # complete this step just as is covered in capture_features.py
        # Check for invalid clouds.
        if sample_cloud_arr.shape[0] == 0:
            print('Invalid cloud detected')
            try_count += 1
        else:
            sample_was_good = True

        # Using hsv instead of RGB
        chists = compute_color_histograms(sample_cloud, using_hsv=True)
        normals = get_normals(sample_cloud)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))
        # labeled_features.append([feature, model_name])    # Not needed, we're not saving the labels as training or model data

        # Make the prediction
        # retrieve the label for the result and add
        # it to the detected_objects_labels list
        prediction = clf.predict(scaler.transform(feature.reshape(1, -1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into Rviz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += 0.4 # TODO: What is this?
        object_markers_pub.publish(make_label(label, label_pos, index))

        # Add the detected object to the list of detected objects
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

        # Do some logging
        rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

        # Publish the list of detected objects
        # This is the output you'll need to complete the upcoming project!
        detected_objects_pub.publish(detected_objects)



    # Publish the list of detected objects

if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node('Object Recognition', anonymous=True)

    # Create Subscribers
    pcl_sub = rospy.Subscriber('/sensor_stick/point_cloud',
    pc2.PointCloud2, pcl_callback, queue_size=1)

    # Create Publishers
    # here you need to create two publishers + 2 previous publishers from exercise 2
    # The new publishers, call them object_markers_pub and detected_objects_pub
    # Have them publish to "/object_markers" and "/detected_objects" with 
    # Message Types "Marker" and "DetectedObjectsArray" , respectively
    pcl_objects_pub = rospy.Publisher('pcl_objects', PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher('/pcl_table', PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher('/pcl_cluster', PointCloud2, queue_size=1)
    object_markers_pub = rospy.Publisher('/object_markers', Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher('/detected_objects', DetectedObjectsArray, queue_size=1)

    # Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
