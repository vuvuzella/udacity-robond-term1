import tensorflow as tf

saver = tf.train.Saver()
save_file = './train_model.cpkt'

with tf.Session() as sess:
    saver.restore(sess, save_file)

    # previous code for loading test data from mnist, see save_trained_model.py
    test_accuracy = sess.run(
    accuracy,
    feed_dict={features: mnist.test.images, labels: mnist.test.labels})

print('Test Accuracy: {}'.format(test_accuracy))