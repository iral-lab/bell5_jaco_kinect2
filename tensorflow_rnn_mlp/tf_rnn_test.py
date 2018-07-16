#!/usr/bin/env python

import numpy as np
import tensorflow as tf
from tensorflow.python import debug as tf_debug
from classifier import get_rnn_cost

data_1 = [
	[[1,0,0,0,0], [1,0,0,0,0],[1,0,0,0,0],[1,0,0,0,0],[1,0,0,0,0],[1,0,0,0,0],[1,0,0,0,0],[1,0,0,0,0],[1,0,0,0,0],[1,0,0,0,0],[1,0,0,0,0],[1,0,0,0,0],[1,0,0,0,0],[1,0,0,0,0],[1,0,0,0,0],[1,0,0,0,0],[1,0,0,0,0],[1,0,0,0,0],[1,0,0,0,0],[1,0,0,0,0],],
]


label_1 = [
	[2, 7000, 6500, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
]

class_length = len(label_1[0])

n_sequences = len(data_1)
sequence_length = len(data_1[0])
frame_length = len(data_1[0][0])

if len(data_1) <> len(label_1):
	print "invalid data/label"
	exit()



X = tf.placeholder('float', [n_sequences, sequence_length, frame_length], name = 'X_input')
Y = tf.placeholder('float', [n_sequences, class_length], name = 'Y_label')

n_hidden = 2 * frame_length
weights = {
    'out': tf.Variable(tf.random_normal([n_hidden, class_length]))
}
biases = {
    'out': tf.Variable(tf.random_normal([class_length]))
}
#init_state = tf.zeros([n_hidden, frame_length])

rnn_cell = tf.contrib.rnn.LSTMCell(n_hidden)
outputs, states = tf.nn.dynamic_rnn(rnn_cell, X, dtype=tf.float32)
pred = tf.nn.relu(tf.add(tf.matmul(outputs[-1], weights['out']), biases['out']))

l2_distance = tf.sqrt( tf.reduce_sum(tf.square(tf.subtract(pred, Y)), reduction_indices=1))
penalty_sum = tf.reduce_sum(l2_distance)
cost = penalty_sum

learning_rate = 0.001
optimizer = tf.train.AdamOptimizer(learning_rate=learning_rate).minimize(cost)
	
with tf.Session() as sess: # create a session to evaluate the symbolic expressions
	init = tf.global_variables_initializer()
	sess.run(init)
	print "\nrnn:\n", sess.run(outputs, feed_dict = {X: data_1, Y: label_1})
	print "\npred:\n", sess.run(pred, feed_dict = {X: data_1, Y: label_1})
	
	for i in xrange(100):
		_,c = sess.run([optimizer, cost], feed_dict={X:data_1, Y: label_1})
		print i,"cost:", c
	
	
	
	
	
