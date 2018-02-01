import math
import tensorflow as tf

def batches(batch_size, features, labels):
    """
    Create batches of features and labels
    :param batch_size: The batch size
    :param features: List of features
    :param labels: List of labels
    :return: Batches of (Features, Labels)
    """
    # assert len(features) == len(labels)
    # outout_batches = []
    
    # sample_size = len(features)
    # for start_i in range(0, sample_size, batch_size):
    #     end_i = start_i + batch_size
    #     batch = [features[start_i:end_i], labels[start_i:end_i]]
    #     outout_batches.append(batch)
    # return outout_batches

    assert len(features) == len(labels)
    outout_batches = []
    
    sample_size = len(features)
    for start_i in range(0, sample_size, batch_size):
        end_i = start_i + batch_size
        batch = [features[start_i:end_i], labels[start_i:end_i]]
        outout_batches.append(batch)

    return outout_batches

def get_weights(n_features, n_classes):
    return tf.Variable(tf.random_normal([n_features, n_classes]))

def get_bias(n_classes):
    return tf.Variable(tf.random_normal([n_classes]))

def linear(x, w, b):
    return tf.add(tf.matmul(x, w), b)

def loss(logits, labels):
    return tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(logits=logits, labels=labels))

def optimize(learn_rate, loss_fn):
    return tf.train.GradientDescentOptimizer(learning_rate=learn_rate).minimize(loss_fn)
