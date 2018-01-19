# histogram_action.py
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np
import cv2

# Define a function to compute color histogram features  
def color_hist(img, nbins=32, bins_range=(0, 256)):
    # Convert from RGB to HSV using cv2.cvtColor()
    # Compute the histogram of the HSV channels separately
    # Concatenate the histograms into a single feature vector
    # Normalize the result
    # Return the feature vector
    img_hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    h_hist = np.histogram(img_hsv[:, :, 0], bins=nbins, range=bins_range)
    s_hist = np.histogram(img_hsv[:, :, 1], bins=nbins, range=bins_range)
    v_hist = np.histogram(img_hsv[:, :, 2], bins=nbins, range=bins_range)
    bin_edges = h_hist[1]
    bin_centers = (bin_edges[1:] + bin_edges[0:len(bin_edges)-1]) / 2
    hist_features = np.concatenate((h_hist[0], s_hist[0], v_hist[0])).astype(np.float64)
    norm_features = hist_features / np.sum(hist_features)
    return norm_features

# filename = 'can.png'
# image = mpimg.imread(filename)
# plt.imshow(image)

# Take histograms in R, G, B
# returns a tuple of two arrays. 
# r_hist[0] contains the counts in each of the bins and r_hist[1] contains
# the bin edges (so it is one element longer than r_hist[0])
# r_hist = np.histogram(image[:, :, 0], bins=32, range=(0, 256))
# g_hist = np.histogram(image[:, :, 1], bins=32, range=(0, 256))
# b_hist = np.histogram(image[:, :, 2], bins=32, range=(0, 256))

# Generate the bin centers
# bin_edges = r_hist[1]
# bin_centers = (bin_edges[1:] + bin_edges[0:len(bin_edges)-1]) / 2

# hist_features = np.concatenate((r_hist[0], g_hist[0], b_hist[0])).astype(np.float64)
# norm_features = hist_features / np.sum(hist_features)


if __name__ == '__main__':

    filename = 'can.png'
    image = mpimg.imread(filename)

    # # Plot it in pyplot
    # fig = plt.figure(figsize=(12,3))
    # plt.subplot(131)
    # plt.bar(bin_centers, r_hist[0])
    # plt.xlim(0, 256)
    # plt.title('R Histogram')
    # plt.subplot(132)
    # plt.bar(bin_centers, g_hist[0])
    # plt.xlim(0, 256)
    # plt.title('G Histogram')
    # plt.subplot(133)
    # plt.bar(bin_centers, b_hist[0])
    # plt.xlim(0, 256)
    # plt.title('B Histogram')
    # plt.show()
    feature_vec = color_hist(image, nbins=32, bins_range=(0, 256))

    # Plot a figure with all three bar charts
    if feature_vec is not None:
        fig = plt.figure(figsize=(12,6))
        plt.plot(feature_vec)
        plt.title('HSV Feature Vector', fontsize=30)
        plt.tick_params(axis='both', which='major', labelsize=20)
        fig.tight_layout()
    else:
        print('Your function is returning None...')
    
    plt.show()