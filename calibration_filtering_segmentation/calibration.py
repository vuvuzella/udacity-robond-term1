# Note: This does not run locally.
import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import glob

# perpare object points
nx = 8
ny = 6

# Make a list of calibration images
images = glob.glob("./Cal*.jpg")

# Select any index to grab an image from the list
idx = -1
# Read in the image
img = mpimg.imread(images[idx])

# Convert to grayscale
gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

# Find the chessboard corners
ret, corners = cv2.findChessboardCorners(gray, (nx, ny), None)

# If found, draw the corners
if ret == True:
    # Draw and display the corners
    cv2.drawChessboardCorners(img, (nx, ny), corners, ret)
    plt.imshow(img)