import tensorflow as tf

# The file path to save
save_file = './model.ckpt'

# Two tensor variables
weights = tf.Variable(tf.truncated_normal([2, 3]))
bias = tf.Variable(tf.truncated_normal([3]))

# Class used to save and/or restore tensor variables

saver = tf.train.Saver()

with tf.Session() as sess:
    sess.run(tf.global_variables_initializer())

    print('Weights: ')
    print(sess.run(weights))
    print('Bias:')
    print(sess.run(bias))

    saver.save(sess, save_file)