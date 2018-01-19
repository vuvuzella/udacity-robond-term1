#!/usr/bin/env python
# Import modules
import rospy
import pcl
import numpy as np
from pcl_helper import *

# TODO: Define functions as required

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

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
    axis_min = 0.6
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

    # TODO: Euclidean Clustering

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately

    # Convert PCL data to ROS messages
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cloud_table = pcl_to_ros(cloud_table)

    # Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)

if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node('clustering', anonymous=True)

    # Create Subscribers
    # Subscribe to '/sensor_stick/point_cloud' topic
    # Message data is apoint cloud data, is passed to pcl_callback() for processing
    pcl_sub = rospy.Subscriber('/sensor_stick/point_cloud', pc2.PointCloud2, pcl_callback, queue_size=1)

    # Create Publishers
    # 2 new publishers to publish the point cloud data for the table
    # and for the objects on the table, called pcl_table and pcl_objects, 
    # respectively
    pcl_objects_pub = rospy.Publisher('pcl_objects', PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher('/pcl_table', PointCloud2, queue_size=1)

    # Initialize color_list
    get_color_list.color_list = []

    # Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()