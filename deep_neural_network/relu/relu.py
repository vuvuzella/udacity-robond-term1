import tensorflow as tf

if __name__ == '__main__':
    output = None
    hidden_layer_weights = [
        [0.1, 0.2, 0.4],
        [0.4, 0.6, 0.6],
        [0.5, 0.9, 0.1],
        [0.8, 0.2, 0.8]
    ]
    out_weights = [
        [0.1, 0.6],
        [0.2, 0.1],
        [0.7, 0.9]
    ]

    # Weights and Biases
    weights = [
        tf.Variable(hidden_layer_weights),
        tf.Variable(out_weights)
    ]

    biases = [
        tf.Variable(tf.zeros(3)),
        tf.Variable(tf.zeros(2))
    ]

    # Input
    features = tf.Variable([
        [ 1.0,  2.0,  3.0,  4.0],
        [-1.0, -2.0, -3.0, -4.0],
        [11.0, 12.0, 13.0, 14.0]])

    # TODO: Create the model
    hidden_layer = tf.add(tf.matmul(features, weights[0]), biases[0])
    hidden_layer = tf.matmul(tf.nn.relu(hidden_layer), weights[1])
    relu = tf.add(hidden_layer, biases[1])

    init = tf.global_variables_initializer()

    with tf.Session() as sess:
        sess.run(init)
        output = sess.run(relu)

    print(output)