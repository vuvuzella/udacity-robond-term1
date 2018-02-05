import tensorflow as tf
import os

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'    # disable debug messages for each tf session run

tf.reset_default_graph()

save_file = './finetuning.cpkt'

# Two tensor variable weights and bias
weights = tf.Variable(tf.random_normal([2, 3]), name='weights_0')
bias = tf.Variable(tf.random_normal([3]), name='bias_0')

saver = tf.train.Saver()

# Print the name of weights
print('Save weights: {}'.format(weights.name))
print('Save labels: {}'.format(bias.name))

with tf.Session() as sess:
    sess.run(tf.global_variables_initializer())
    saver.save(sess, save_file)

# Remove the previous weights and bias
tf.reset_default_graph()

# Two Variables: weights and bias
bias = tf.Variable(tf.truncated_normal([3]), name='bias_0')
weights = tf.Variable(tf.truncated_normal([2, 3]), name='weights_0')

saver = tf.train.Saver()

# Print the name of Weights and Bias
print('Load Weights: {}'.format(weights.name))
print('Load Bias: {}'.format(bias.name))

with tf.Session() as sess:
    # Load the weights and bias - No error if tf variables have name parameters set
    saver.restore(sess, save_file)

print('Loaded Weights and Bias successfully')

