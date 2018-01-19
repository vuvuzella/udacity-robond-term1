# Import PCL module
import pcl

# Load Point Cloud file
cloud = pcl.load_XYZRGB('tabletop.pcd')


# Voxel Grid filter
vox = cloud.make_voxel_grid_filter()

# Choose a voxel (also known as a leaf) size
# Note: this (1) is a poor choice of leaf size
# Experiment and find the appropriate size
LEAF_SIZE = 0.01

# Set the voxel or leaf size
vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)

# Call the filter function to obtain the resultant downsampled point cloud
cloud_filtered = vox.filter()
filename = 'voxel_downsampled.pcd'
pcl.save(cloud_filtered, filename)

# PassThrough filter
# Create a PassThrough filter object
passthrough = cloud_filtered.make_passthrough_filter()

# Assign axis and range to the passthrough filter object
filter_axis = 'z'
passthrough.set_filter_field_name(filter_axis)
axis_min = 0.6
axis_max = 1.1
passthrough.set_filter_limits(axis_min, axis_max)

# Finally use the filter function to obtain the resultant point cloud
cloud_filtered = passthrough.filter()
filename = 'pass_through_filtered.pcd'
pcl.save(cloud_filtered, filename)


# RANSAC plane segmentation
# Create the segmentation object
seg = cloud_filtered.make_segmenter()

# Set the model you wish to fit
seg.set_model_type(pcl.SACMODEL_PLANE)
seg.set_method_type(pcl.SAC_RANSAC)

# Max distance for a point to be considered fitting the model
# Experiment with different values for max_distance
# for segmenting the table
max_distance = 0.01
seg.set_distance_threshold(max_distance)

# Call the segment function to obtain set of inlier indices and model coefficients
inliers, coefficients = seg.segment()

# ExtractIndices Filter
# Extract inliers
# changing the negative flag to true can extract the outliers
extracted_inliers = cloud_filtered.extract(inliers, negative=False)
filename = 'extracted_inliers.pcd'
pcl.save(extraced_inliers, filename)

# Save pcd for table
# pcl.save(cloud, filename)


# Extract outliers
# changing the negative flag to true can extract the outliers
extracted_inliers = cloud_filtered.extract(inliers, negative=True)
filename = 'extracted_outliers.pcd'
pcl.save(extraced_inliers, filename)

# Optional, perform outlier removal for aditional noise removal
# # PCL's StatisticalOutlierRemoval filter
# # Check the neighborhood of each point and remove which points do not
# # meet certain criteria
# 

# load new image with noise:
p = pcl.load('table_scene_lms400.pcd')
filename = 'table_scene_lms400_inliers.pcd'

# Much like the previous filters, we start by creating a filter object:
outlier_filter = cloud_filtered.make_statistical_outlier_filter()

# Set the number of neighboring points to analyze for any given point
outlier_filter.set_mean_k(50)

# Set threshold scale factor
x = 1.0

# Any point with a mean distance larger than global (mean distance + x * std_dev)
# will be considered an outlier
outlier_filter.set_std_dev_mul_thresh(x)

# Finally call the filter function for magic
cloud_filtered = outlier_filter.filter()
pcl.save(cloud_filtered, filename)

outlier_filter.set_negative(True)
cloud_filtered = outlier_filter.filter()
filename = 'table_scene_lms400_outliers.pcd'
pcl.save(cloud_filtered, filename)





