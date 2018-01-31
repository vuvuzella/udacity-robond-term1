# Solution is available in the other "quiz_solution.py" tab
import tensorflow as tf

def get_weights(n_features, n_labels):
    """
    Return TensorFlow weights
    :param n_features: Number of features
    :param n_labels: Number of labels
    :return: TensorFlow weights
    """
    # Return weights
    return tf.Variable(tf.truncated_normal((n_features, n_labels)))


def get_biases(n_labels):
    """
    Return TensorFlow bias
    :param n_labels: Number of labels
    :return: TensorFlow bias
    """
    # Return biases
    # return tf.Variable(tf.truncated_normal((n_labels,)))  # original answer
    return tf.Variable(tf.zeros(n_labels))  # quiz answer


def linear(inp, w, b):
    """
    Return linear function in TensorFlow
    :param inp: TensorFlow input
    :param w: TensorFlow weights
    :param b: TensorFlow biases
    :return: TensorFlow linear function
    """
    # Linear Function (xW + b)
    return tf.add(tf.matmul(inp, w), b)