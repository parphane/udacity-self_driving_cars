from pprint import pprint
import numpy as np
import tensorflow.compat.v1 as tf
tf.disable_v2_behavior()
#import tensorflow as tf

#from tensorflow.examples.tutorials.mnist import input_data

n_input = 784  # MNIST data input (img shape: 28*28)
n_classes = 10  # MNIST total classes (0-9 digits)

# Import MNIST data
#mnist = input_data.read_data_sets('/datasets/ud730/mnist', one_hot=True)
# Updated for TF2
mnist = tf.keras.datasets.mnist

# The features are already scaled and the data is shuffled
#train_features = mnist.train.images
#test_features = mnist.test.images

#train_labels = mnist.train.labels.astype(np.float32)
#test_labels = mnist.test.labels.astype(np.float32)

(train_features, train_labels), (test_features, test_labels) = mnist.load_data()

# Weights & bias
weights = tf.Variable(tf.random_normal([n_input, n_classes]))
bias = tf.Variable(tf.random_normal([n_classes]))

# Features and Labels
features = tf.placeholder(tf.float32, [None, n_input])
labels = tf.placeholder(tf.float32, [None, n_classes])

# 4 Samples of features
example_features = [
    ['F11','F12','F13','F14'],
    ['F21','F22','F23','F24'],
    ['F31','F32','F33','F34'],
    ['F41','F42','F43','F44']]
# 4 Samples of labels
example_labels = [
    ['L11','L12'],
    ['L21','L22'],
    ['L31','L32'],
    ['L41','L42']]

def batches(batch_size, features, labels):
    """
    Create batches of features and labels
    :param batch_size: The batch size
    :param features: List of features
    :param labels: List of labels
    :return: Batches of (Features, Labels)
    """
    assert len(features) == len(labels)
    # TODO: Implement batching
    output_batches = []

    sample_size = len(features)
    for start_i in range(0, sample_size, batch_size):
        end_i = start_i + batch_size
        batch = [features[start_i:end_i], labels[start_i:end_i]]
        output_batches.append(batch)
    return output_batches


# PPrint prints data structures like 2d arrays, so they are easier to read
example_batches = batches(3, example_features, example_labels)
pprint(example_batches)
