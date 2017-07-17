import sys, os
import tensorflow as tf
from generator import HEADER_DIVIDER, SKELETON_MARKER, LENGTHS_HEADER, PERMUTATIONS, get_skeleton_points

MAX_CENTROIDS = 20
MAX_LENGTHS = 6

INPUT_FOLDER = 'clouds/'

def get_label(lengths_line):
	lengths = [int(x) for x in lengths_line[len(LENGTHS_HEADER):].split("\t")]
	num_lengths = len(lengths)
	while len(lengths) < MAX_LENGTHS:
		lengths.append(0.0)
	return tuple([num_lengths] + lengths)
	

def naive_frame_reader(files):
	for file in files:
		with open(INPUT_FOLDER+file, 'r') as handle:
			skeleton_frame = None
			label = None
			input = handle.readline()
			while input:
				line = input.strip()
				input = handle.readline()
				
				if "" == line:
					continue
				
				if HEADER_DIVIDER == line and skeleton_frame:
					if skeleton_frame and label:
						yield (label, skeleton_frame)
					else:
						print "Missing skeleton_frame or label"
					skeleton_frame = None
					label = None
					continue
				
				if SKELETON_MARKER in line:
					skeleton_frame = get_skeleton_points(line)
				
				if LENGTHS_HEADER in line:
					label = get_label(line)

if '__main__' == __name__:
	
	input_files = os.listdir(INPUT_FOLDER)
	
	n_nodes_hl1 = 50
	n_nodes_hl2 = 50
	n_nodes_hl3 = 50
	
	n_classes = len(input_files) * PERMUTATIONS # robots * variations, # of files * # of permutations
	
	x = tf.placeholder('float', [3, MAX_CENTROIDS])
	y = tf.placeholder('float', [MAX_LENGTHS, None])
	
	data = []# <<<
	
	hidden_1 = {'weights' : tf.Variable(tf.random_normal([MAX_CENTROIDS, n_nodes_hl1])), 'biases' : tf.Variable(tf.random_normal([n_nodes_hl1,1]))}
	hidden_2 = {'weights' : tf.Variable(tf.random_normal([n_nodes_hl1, n_nodes_hl2])), 'biases' : tf.Variable(tf.random_normal([n_nodes_hl2,1]))}
	hidden_3 = {'weights' : tf.Variable(tf.random_normal([n_nodes_hl2, n_nodes_hl3])), 'biases' : tf.Variable(tf.random_normal([n_nodes_hl3,1]))}
	
	output = {'weights' : tf.Variable(tf.random_normal([n_nodes_hl3, n_classes])), 'biases' : tf.Variable(tf.random_normal([n_classes,1]))}
	
	layer_1 = tf.add(tf.matmul(data, hidden_1['weights']), hidden_1['biases'])
	layer_1 = tf.nn.relu(layer_1)
	
	layer_2 = tf.add(tf.matmul(layer_1, hidden_2['weights']), hidden_2['biases'])
	layer_2 = tf.nn.relu(layer_2)
	
	layer_3 = tf.add(tf.matmul(layer_2, hidden_3['weights']), hidden_3['biases'])
	layer_3 = tf.nn.relu(layer_3)
	
	# for frame in naive_frame_reader(input_files):
	# 	print frame
	# 	break
	













