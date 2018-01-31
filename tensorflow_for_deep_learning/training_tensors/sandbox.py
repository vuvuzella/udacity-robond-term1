import tensorflow as tf
from tensorflow.examples.tutorials.mnist import input_data
from quiz import get_weights, get_biases, linear

def mnist_features_labels(n_labels):
    '''
    Gets the first <n> labels from the MNIST dataset
    :param n_labels: Number of labels to use
    :return: Tuple of feature list and label list
    '''
    mnist_features = []
    mnist_labels = []

    mnist = input_data.read_data_sets('./datasets/ud730/mnist', one_hot=True)

    # In order to make quizzes run faster, we're only looking at 10000 images
    for feature, label in zip(*mnist.train.next_batch(10000)):
        # Add features and labels if it's for the first nth labels
        if label[:n_labels].any():
            mnist_features.append(feature)
            mnist_labels.append(label[:n_labels])

    return mnist_features, mnist_labels

if __name__ == '__main__':

    # Number of features (28 * 28 image is 784 features)
    n_features = 784
    # Number of labels
    n_labels = 3

    # Features and Labels
    features = tf.placeholder(tf.float32)
    labels = tf.placeholder(tf.float32)

    # Weights and biases
    weights = get_weights(n_features, n_labels)
    biases = get_biases(n_labels)

    # Linear Function
    logits = linear(features, weights, biases)

    # Training Data
    train_features, train_labels = mnist_features_labels(n_labels)

    with tf.Session() as sess:
        # Initialize session variables
        sess.run(tf.global_variables_initializer())
    
        # Softmax
        prediction = tf.nn.softmax(logits)  # create probabilities from logit scores

        # Cross entropy
        # This quantfies how far off the predictions were
        cross_entropy = -tf.reduce_sum(labels * tf.log(prediction), reduction_indices=1)

        # Training Loss
        loss = tf.reduce_mean(cross_entropy)

        # Rate at which the weights are changed
        learning_rate = 0.08

        # Gradient descent
        # Method used to train the model
        optimizer = tf.train.GradientDescentOptimizer(learning_rate).minimize(loss)

        # Run the optimizer and get the loss
        _, l = sess.run([optimizer, loss], feed_dict={features: train_features, labels: train_labels})

    print('Loss: {}'.format(l))
