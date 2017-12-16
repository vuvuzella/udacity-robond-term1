import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

# def obstacle_color_thresh(img):
#     # get the non-obstacle color threshold of the same image
#     binary_image = color_thresh(img)
#     # create a transformed image of ones, which will be used to XOR to the original image        
#     black_mask = perspect_transform(np.ones_like(img[:, :, 0]), source, destination)
# 
#     # xor the black mask with the binary image 
#     binary_image = np.bitwise_xor(binary_image, black_mask)
#     # Return the binary image
#     return binary_image

#
def rock_color_thresh(img):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > 120) \
                & (img[:,:,0] < 210) \
                & (img[:,:,1] > 120) \
                & (img[:,:,1] < 210) \
                & (img[:,:,2] < 80)
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    img = Rover.img
    # 1) Define source and destination points for perspective transform

    # Define calibration box in source (actual) and destination (desired) coordinates
    # These source and destination points are defined to warp the image
    # to a grid where each 10x10 pixel square represents 1 square meter
    # The destination box will be 2*dst_size on each side
    dst_size = 5 
    # Set a bottom offset to account for the fact that the bottom of the image 
    # is not the position of the rover but a bit in front of it
    # this is just a rough guess, feel free to change it!
    bottom_offset = 6

    source_lowerLeft = [14, 140]
    source_lowerRight = [301 ,140]
    source_upperRight = [200, 96]
    source_upperLeft = [118, 96]

    source = np.float32([source_lowerLeft, 
                         source_lowerRight,
                         source_upperRight,
                         source_upperLeft])

    xMid = img.shape[1] / 2 # get the x-center of the image
    yUpper = img.shape[0] - 2*dst_size - bottom_offset
    yLower = img.shape[0] - bottom_offset

    destination = np.float32([[xMid - dst_size, yLower],
                      [xMid + dst_size, yLower],
                      [xMid + dst_size, yUpper], 
                      [xMid - dst_size, yUpper]])

    # 2) Apply perspective transform
    transform = perspect_transform(img, source, destination)

    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    # navi_thresh = color_thresh(img) * 255
    # navi_thresh = perspect_transform(navi_thresh, source, destination)
    navi_thresh = color_thresh(transform) * 255
    rock_thresh = rock_color_thresh(transform) * 255
    black_mask = perspect_transform(np.ones_like(transform[:,:,0]), source, destination)
    obstacle_thresh = np.bitwise_xor(navi_thresh, black_mask) * 255

    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    Rover.vision_image[:,:,0] = obstacle_thresh
    Rover.vision_image[:,:,1] = rock_thresh
    Rover.vision_image[:,:,2] = navi_thresh

    # 5) Convert map image pixel values to rover-centric coords
    navi_rover_x, navi_rover_y = rover_coords(navi_thresh)
    obstacle_rover_x, obstacle_rover_y = rover_coords(obstacle_thresh)
    rock_rover_x, rock_rover_y = rover_coords(rock_thresh)

    # 6) Convert rover-centric pixel values to world coordinates
    yaw = Rover.yaw
    xpos = Rover.pos[0]
    ypos = Rover.pos[1]

    navi_world_x, navi_world_y = pix_to_world(navi_rover_x, navi_rover_y, xpos, ypos, yaw, 200, 10)
    obstacle_world_x, obstacle_world_y = pix_to_world(obstacle_rover_x, \
    obstacle_rover_y, xpos, ypos, yaw, 200, 10)
    rock_world_x, rock_world_y = pix_to_world(rock_rover_x, rock_rover_y, xpos, ypos, yaw, 200, 10)

    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    if Rover.roll > 270:
        roll = 360 - Rover.roll
    else:
        roll = Rover.roll
    if Rover.pitch > 270:
        pitch = 360 - Rover.pitch
    else:
        pitch = Rover.pitch
    if roll < 1 and pitch < 1:
        Rover.worldmap[obstacle_world_y, obstacle_world_x, 0] += 1
        Rover.worldmap[navi_world_y, navi_world_x, 2] += 1
        Rover.worldmap[navi_world_y, navi_world_x, 0] = 0
    Rover.worldmap[rock_world_y, rock_world_x, 1] += 1

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    Rover.nav_dists, Rover.nav_angles = to_polar_coords(navi_rover_x, navi_rover_y)
    Rover.rock_dists, Rover.rock_angles = to_polar_coords(rock_rover_x, rock_rover_y)
    
    if len(Rover.nav_angles) > Rover.go_forward:
        Rover.nav_last_angle_seen = Rover.nav_angles
    
    
    return Rover