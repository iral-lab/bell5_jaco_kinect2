import sys, os
import tensorflow as tf
from generator import HEADER_DIVIDER, SKELETON_MARKER, LENGTHS_HEADER, MAX_CENTROIDS, MAX_LINKS, PERMUTATIONS, get_skeleton_points

MAX_LENGTHS = 6

INPUT_FOLDER = 'clouds/'

def get_label(lengths_line):
	lengths = [int(x) for x in lengths_line[len(LENGTHS_HEADER):].split("\t")]
	num_lengths = len(lengths)
	while len(lengths) < MAX_LENGTHS:
		lengths.append(0.0)
	return _pad_label([num_lengths] + lengths)
	

def _flatten_and_pad_points(skeleton_frame):
	flat = [item for sublist in skeleton_frame for item in sublist]
	while len(flat) < MAX_CENTROIDS * 3:
		flat.append(0.0)
	return flat

def prepare_skeleton(line):
	return _flatten_and_pad_points(get_skeleton_points(line))

def _pad_label(lengths):
	new = list(lengths)[:]
	while len(new) < MAX_LINKS+1:
		new.append(0.0)
	return new

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
					skeleton_frame = prepare_skeleton(line)
				
				if LENGTHS_HEADER in line:
					label = get_label(line)

def xy_batcher(files):
	batch_size = 10
	batch = []
	for frame in naive_frame_reader(input_files):
		batch.append(frame)
		if len(batch) == batch_size:
			yield batch
			batch = []
	if len(batch) > 0:
		yield batch


def rnn_model(data):

	n_nodes_hl1 = 50
	n_nodes_hl2 = 50
	n_nodes_hl3 = 50

	hidden_1 = {'weights' : tf.Variable(tf.random_normal([MAX_CENTROIDS, n_nodes_hl1])), 'biases' : tf.Variable(tf.random_normal([n_nodes_hl1]))}
	hidden_2 = {'weights' : tf.Variable(tf.random_normal([n_nodes_hl1, n_nodes_hl2])), 'biases' : tf.Variable(tf.random_normal([n_nodes_hl2]))}
	hidden_3 = {'weights' : tf.Variable(tf.random_normal([n_nodes_hl2, n_nodes_hl3])), 'biases' : tf.Variable(tf.random_normal([n_nodes_hl3]))}
	
	output_layer = {'weights' : tf.Variable(tf.random_normal([n_nodes_hl3, n_classes])), 'biases' : tf.Variable(tf.random_normal([n_classes]))}
	
	layer_1 = tf.add(tf.matmul(data, hidden_1['weights']), hidden_1['biases'])
	layer_1 = tf.nn.relu(layer_1)
	
	layer_2 = tf.add(tf.matmul(layer_1, hidden_2['weights']), hidden_2['biases'])
	layer_2 = tf.nn.relu(layer_2)
	
	layer_3 = tf.add(tf.matmul(layer_2, hidden_3['weights']), hidden_3['biases'])
	layer_3 = tf.nn.relu(layer_3)

	output = tf.add(tf.matmul(layer_3, output_layer['weights']), output_layer['biases'])
	return output


def train_neural_network(files, x, y, test_labels, test_data):
	prediction = rnn_model(x)
	cost = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(logits=prediction, labels=y))
	optimizer = tf.train.AdamOptimizer().minimize(cost)
	n_epochs = 10
	batcher = xy_batcher(files)
	with tf.Session() as sess:
		sess.run(tf.global_variables_initializer())
		for epoch in range(n_epochs):
			
			batch = batcher.next()
			if not batch:
				break
			epoch_x,epoch_y = batch
			_,c = sess.run([optimizer, cost], feed_dict = {x: epoch_x, y: epoch_y})
			print "Epoch",epoch,"completed of",n_epochs,"loss:",c
		correct = tf.equal(tf.argmax(prediction,1), tf.argmax(y, 1))
		accuracy = tf.reduce_mean(tf.cast(correct, 'float'))
		print "Accuracy:",accuracy.eval({x:test_data, y:test_labels})
			
			

if '__main__' == __name__:
	
	input_files = os.listdir(INPUT_FOLDER)
	print len(input_files)
	
	n_classes = len(input_files) * PERMUTATIONS # robots * variations, # of files * # of permutations
	
	x = tf.placeholder('float', [3, MAX_CENTROIDS])
	y = tf.placeholder('float', [MAX_LENGTHS, None])
	
	test_labels = [_pad_label((3,3500,5000,2500)), ]
	test_data = [prepare_skeleton("#Skeleton#110,-2181,2125	776,-256,728	1452,-1181,2161	-920,-3102,2254	1999,-366,1674	272,-230,173	-322,-2666,2087	2288,-632,2110	-365,-2258,3122	579,-1719,2168	1441,-318,1257")]

	#print test_labels, test_data

	train_neural_network(input_files, x, y, test_labels, test_data)











