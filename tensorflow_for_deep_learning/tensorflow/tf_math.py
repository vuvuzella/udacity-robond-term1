import tensorflow as tf

# Convert the ntaive constants to tensorflow data types
x = tf.constant(10)
y = tf.constant(2)
z = tf.subtract(tf.divide(x, y), tf.cast(tf.constant(1), tf.float64))

with tf.Session() as sess:
    output = sess.run(z)
    print('output: ' + str(output))
