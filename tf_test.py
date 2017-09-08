#!/usr/bin/env python

import numpy as np
import tensorflow as tf
from tensorflow.python import debug as tf_debug
from classifier import MAX_LINKS, MAX_CENTROIDS, load_data_cache, naive_batcher


class_length = 9 #MAX_LINKS + 1
n_input = MAX_CENTROIDS * 3


Y = tf.placeholder('float', [None, class_length])
X = tf.placeholder('float', [None, class_length])

# data_cache, label_cache = load_data_cache()

# batch = naive_batcher(data_cache, label_cache)
# epoch_x, epoch_y = batch.next()
# print epoch_y

prediction = np.array([[2,1,2,9,9,6,5,4,4], [4,4,3,2,1,3,4,6,7], [3,1,9,9,9,4,3,3,3], [699862.3125, 777693.3125, 1971025.875, -133910.421875, -1138590.875, 53177.03515625, 1322253.875, -1262390.125, -425135.25]])

epoch_y = np.array([[3,1,2,3,0,0,0,0,0], [4,1,2,3,4,0,0,0,0], [2,1,2,0,0,0,0,0,0], [3, 2000, 6500, 4500, 0.0, 0.0, 0.0, 0.0, 0.0]])

link_count_importance = 100

correct_noop = tf.transpose(tf.transpose(Y))
prediction_noop = tf.transpose(tf.transpose(X))

correct_transpose = tf.transpose(Y)
prediction_transpose = tf.transpose(X)

correct_link_counts = tf.cast(tf.gather(correct_transpose, 0), tf.int32)
predicted_link_counts = tf.cast(tf.gather(prediction_transpose, 0), tf.int32)


correct_length_mask = tf.sequence_mask(correct_link_counts, class_length - 1, tf.int32)

shift_mask = tf.cast(tf.zeros([tf.shape(correct_transpose)[1],1]), tf.int32)

shifted_correct_length_mask = tf.cast(tf.concat([shift_mask, correct_length_mask], 1), tf.float32)
reshaped_shifted_correct_length_mask = tf.reshape(shifted_correct_length_mask, [-1])

reshaped_correct = tf.reshape(Y, [-1])
reshaped_prediction = tf.reshape(X, [-1])

correct_link_lengths = tf.reshape(reshaped_shifted_correct_length_mask * reshaped_correct, tf.shape(Y))
predicted_link_lengths = tf.reshape(reshaped_shifted_correct_length_mask * reshaped_prediction, tf.shape(Y))

l2_distance = tf.sqrt( tf.reduce_sum(tf.square(tf.subtract(correct_link_lengths, predicted_link_lengths)), reduction_indices=1))

wrong_link_counts = tf.cast(link_count_importance * tf.abs(correct_link_counts - predicted_link_counts), tf.float32)

penalty = l2_distance + wrong_link_counts

total_penalty = tf.reduce_sum(penalty)

considered_correct = link_count_importance * 0.1

accurate_predictions = tf.cast(tf.less(penalty, considered_correct), tf.int32)

accuracy_ratio = tf.cast(tf.count_nonzero(accurate_predictions), tf.float32) / tf.cast(tf.shape(Y)[0], tf.float32)


with tf.Session() as sess: # create a session to evaluate the symbolic expressions
	
	print "\ntrue input:\n", sess.run(correct_noop, feed_dict = {X: prediction, Y: epoch_y})
	print "\nprediction:\n", sess.run(prediction_noop, feed_dict = {X: prediction, Y: epoch_y})
	
	print "\ntrue trans:\n", sess.run(correct_transpose, feed_dict = {X: prediction, Y: epoch_y})
	print "\nprediction trans:\n", sess.run(prediction_transpose, feed_dict = {X: prediction, Y: epoch_y})
	
	print "\ntrue counts:\n", sess.run(correct_link_counts, feed_dict = {X: prediction, Y: epoch_y})
	print "\nprediction counts:\n", sess.run(predicted_link_counts, feed_dict = {X: prediction, Y: epoch_y})
	
	print "\nmask:\n", sess.run(correct_length_mask, feed_dict = {X: prediction, Y: epoch_y})
	print "\nshift:\n", sess.run(shift_mask, feed_dict = {X: prediction, Y: epoch_y})
	print "\nshifted mask:\n", sess.run(shifted_correct_length_mask, feed_dict = {X: prediction, Y: epoch_y})
	
	
	print "\nreshaped true trans:\n", sess.run(reshaped_correct, feed_dict = {X: prediction, Y: epoch_y})
	print "\nreshaped prediction trans:\n", sess.run(reshaped_prediction, feed_dict = {X: prediction, Y: epoch_y})
	
	print "\ncorrect lengths:\n", sess.run(correct_link_lengths, feed_dict = {X: prediction, Y: epoch_y})
	print "\npredicted lengths:\n", sess.run(predicted_link_lengths, feed_dict = {X: prediction, Y: epoch_y})
	
	print "\ndistance:\n", sess.run(l2_distance, feed_dict = {X: prediction, Y: epoch_y})
	
	print "\nwrong links:\n", sess.run(wrong_link_counts, feed_dict = {X: prediction, Y: epoch_y})
	print "\npenalty:\n", sess.run(penalty, feed_dict = {X: prediction, Y: epoch_y})
	print "\npenalty sum:\n", sess.run(total_penalty, feed_dict = {X: prediction, Y: epoch_y})
	
	print "\naccurate predictions (penalty under",considered_correct,"):\n", sess.run(accurate_predictions, feed_dict = {X: prediction, Y: epoch_y})
	
	print "\naccuracy %:\n", sess.run(accuracy_ratio, feed_dict = {X: prediction, Y: epoch_y})
	
	
