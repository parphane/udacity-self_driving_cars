import tensorflow as tf


# Create TensorFlow object called tensor
hello_constant = tf.constant('Hello World!')

# Deprecated, see https://www.tensorflow.org/guide/migrate
#import tensorflow.compat.v1 as tf
#tf.disable_v2_behavior()
#with tf.Session() as sess:
#    # Run the tf.constant operation in the session
#    output = sess.run(hello_constant)
#    print(output)

@tf.function
def tf_func():
    return hello_constant


out_a = tf_func()
print(out_a)

# A is a 0-dimensional int32 tensor
A = tf.constant(1234)
# B is a 1-dimensional int32 tensor
B = tf.constant([123, 456, 789])
# C is a 2-dimensional int32 tensor
C = tf.constant([[123, 456, 789], [222, 333, 444]])

# No more placeholders in tensorflow 2
#x = tf.placeholder(tf.string)
#with tf.Session() as sess:
#    output = sess.run(x, feed_dict={x: 'Test String', y: 123, z: 45.67})
#    print(output)

@tf.function
def tf_func1(x1):
    return x1


out_a = tf_func1('Hello World!')
print(out_a)

# TODO: Convert the following to TensorFlow:
x = 10
y = 2
z = x/y - 1
# --> Converted...
#x = tf.constant(10)
#y = tf.constant(2)
#z = tf.subtract(tf.divide(x, y), tf.cast(tf.constant(1), tf.float64))

# TODO: Print z from a session as the variable output
#with tf.Session() as sess:
#    output = sess.run(z)
#    print(output)

# With feeding
#x = tf.placeholder(tf.float32)
#y = tf.placeholder(tf.float32)
#c = tf.placeholder(tf.float32)
#z = tf.subtract(tf.divide(x, y), c)

# TODO: Print z from a session as the variable output
#with tf.Session() as sess:
#    output = sess.run(z, feed_dict={x: 10.0, y: 2.0, c: 1.0})
#    print(output)

@tf.function
def tf_func2(x2, y2):
    return x2/y2 - 1.0


out_a = tf_func2(10.0, 2.0)
print(out_a)


