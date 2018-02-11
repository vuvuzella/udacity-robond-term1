import tensorflow as tf
import numpy as np
import math
from tensorflow.examples.tutorials.mnist import input_data

# remove the tensors, useful if there were previous sessions
# tf.reset_default_grapht()

learning_rate = 0.001
n_input = 784   # MNIST data input (img shape = 28 * 28)
n_classes = 10  # MNIST total classes (0-9 digits)

batch_size = 128
n_epochs = 100

# Import MNIST data
mnist = input_data.read_data_sets('.', one_hot=True)

# Features and labels
features = tf.placeholder(tf.float32, [None, n_input])  # vector of features
labels = tf.placeholder(tf.float32, [None, n_classes])

# Weights and biases
weights = tf.Variable(tf.random_normal([n_input, n_classes]))   # 784 x 10 matrix
bias = tf.Variable(tf.random_normal([n_classes]))   # vector, 10 columns, 1 row

# logits = xW + b
logits = tf.add(tf.matmul(features, weights), bias)

# Define loss and optimizer
cost = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(logits=logits, labels=labels))
optimizer = tf.train.GradientDescentOptimizer(learning_rate=learning_rate).minimize(cost)

# Calculate accuracy
correct_prediction = tf.equal(tf.argmax(logits, 1), tf.argmax(labels, 1))
accuracy = tf.reduce_mean(tf.cast(correct_prediction, tf.float32))

# the filename of the trained model to save to
save_file = './train_model.ckpt'
saver = tf.train.Saver()

with tf.Session() as sess:
    # sess.run(tf.global_variables_initializer())

    # Train cycle
    # for epoch in range(n_epochs):
    #     total_batch = math.ceil(mnist.train.num_examples / batch_size)

    #     # Loop over all batches
    #     for i in range(total_batch):
    #         batch_features, batch_labels = mnist.train.next_batch(batch_size)
    #         sess.run(optimizer, feed_dict={features: batch_features, labels: batch_labels})

    #     if epoch % 10 == 0: # epoch is a multiple of 10
    #         valid_accuracy = sess.run(accuracy,
    #             feed_dict={
    #                 features: mnist.validation.images,
    #                 labels: mnist.validation.labels})

    #         print('Epoch: {:<3} - Validation Accuracy: {}'.format(epoch, valid_accuracy))

    # # Save the model
    # saver.save(sess, save_file)
    # print('Trained Model has been saved')

    saver.restore(sess, save_file)
    valid_accuracy = sess.run(accuracy,
        feed_dict={
            features: mnist.validation.images,
            labels: mnist.validation.labels})
    print('Validation Accuracy: {}'.format(valid_accuracy))





