import tensorflow as tf

def run():
    output = None
    x = tf.placeholder(tf.int32)

    with tf.Session() as sess:
        # feed the x tensor with 123
        output = sess.run(x, feed_dict={x: 123})
    return output    

if __name__ == '__main__':
    print(run())