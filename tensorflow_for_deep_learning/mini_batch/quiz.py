import tensorflow as tf
import numpy as np
from tensorflow.examples.tutorials.mnist import input_data
from helper import *

if __name__ == '__main__':
    learning_rate = 0.001
    n_input = 784
    n_classes = 10

    mnist = input_data.read_data_sets('./datasets/ud730/mnist', one_hot=True)

    train_features = mnist.train.images
    test_features = mnist.test.images

    train_labels = mnist.train.labels.astype(np.float)
    test_labels = mnist.test.labels.astype(np.float)

    features = tf.placeholder(tf.float32, [None, n_input])
    labels = tf.placeholder(tf.float32, [None, n_classes])

    weights = get_weights(n_input, n_classes)
    bias = get_bias(n_classes)

    logits = linear(features, weights, bias)

    # Define loss and optimizer
    cost = loss(logits, labels)
    optimizer = optimize(learning_rate, cost)

    # calulate accuracy
    correct_prediction = tf.equal(tf.argmax(logits, 1), tf.argmax(labels, 1))
    accuracy = tf.reduce_mean(tf.cast(correct_prediction, tf.float32))

    # Set batch size
    batch_size = 128
    assert batch_size is not None, 'You must set the batch size'

    init = tf.global_variables_initializer()

    with tf.Session() as sess:
        sess.run(init)

        # TODO: train optimizer on all batches
        # for batch_features, betch_labels in __
        batch_data = batches(batch_size, train_features, train_labels)
        for i in range(len(batch_data)):
            batch_features = batch_data[i][0]
            batch_labels = batch_data[i][1]
            sess.run(optimizer, feed_dict={features: batch_features, labels: batch_labels})

        test_accuracy = sess.run(accuracy, feed_dict={features: test_features, labels: test_labels})

    print('Test Accuracy: {}'.format(test_accuracy))

