import tensorflow as tf

if __name__ == '__main__':
    output = None
    logit_data = [2.0, 1.0, 0.1]
    logits = tf.placeholder(tf.float32)

    softmax = tf.nn.softmax(logit_data)
    
    with tf.Session() as sess:
        output = sess.run(softmax, feed_dict={logits: logit_data})

    print(output)