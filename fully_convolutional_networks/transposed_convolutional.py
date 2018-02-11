import tensorflow as tf
import numpy as np
from one_by_one import custom_init

def upsample(x):
    """
    Apply a 2 times upsample on x and return the result
    x - the input feature
      - 4 rank tensor
    return: TF operation
    """
    shape = x.get_shape()
    input_shape = shape[1]
    kernel_size = (input_shape * 2,input_shape * 2)
    stride = 2
    return tf.contrib.layers.conv2d_transpose(inputs=x,
                                              num_outputs=shape[3],
                                              kernel_size=kernel_size,
                                              stride=stride,
                                              padding='SAME')

if __name__ == '__main__':

    x = tf.constant(np.random.randn(1, 4, 4, 3), dtype=tf.float32)
    conv = upsample(x)

    with tf.Session() as sess:
        sess.run(tf.global_variables_initializer())
        result = sess.run(conv)

        print('Input Shape: {}'.format(x.get_shape()))
        print('Output Shape: {}'.format(result.shape))