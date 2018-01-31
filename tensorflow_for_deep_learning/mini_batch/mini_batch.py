from tensorflow.examples.tutorial.mnist import input_data
import tensorflow as tf
import numpy as np

n_input = 784   # MNIST data input (img shape: 28 * 28)
n_classes = 10 # MNIST total classes

# import mnist data
mnist = input_data.read_data_sets('/datasets/ud730/mnist', one_hot=True)

# The features are already scaled and the data is shuffled
train_features = mnist.train.images
test_features = mnist.test.features

train_labels = mnist.train.labels.astype(np.float32)
test_labels = mnist.test.labels.astype(np.float32)

# Weights and bias
weights = tf.Variable(tf.random_normal([n_input, n_classes]))
bias = tf.Variable(tf.random_normal([n_classes]))