import sys, os
import tensorflow as tf
from generator import HEADER_DIVIDER, SKELETON_MARKER, LENGTHS_HEADER, MAX_CENTROIDS, MAX_LINKS, PERMUTATIONS, get_skeleton_points

N_EPOCHS = 10

INPUT_FOLDER = 'clouds/'


def get_label(lengths_line):
	lengths = [int(x) for x in lengths_line[len(LENGTHS_HEADER):].split("\t")]
	num_lengths = len(lengths)
	while len(lengths) < MAX_LINKS:
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

def naive_frame_reader(file):
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
	batch_size = len(files) / N_EPOCHS
	print "batch size:",batch_size
	labels_batch = []
	data_batch = []
	for file in files:
		for frame in naive_frame_reader(file):
			label,data = frame
			labels_batch.append(label)
			data_batch.append(data)
			if len(labels_batch) == batch_size:
				yield (data_batch, labels_batch)
				labels_batch = []
				data_batch = []
		if len(labels_batch) > 0:
			yield (data_batch, labels_batch)


def rnn_model(data):

	n_nodes_hl1 = 50
	n_nodes_hl2 = 50
	n_nodes_hl3 = 50

	hidden_1 = {'weights' : tf.Variable(tf.random_normal([MAX_CENTROIDS * 3, n_nodes_hl1])), 'biases' : tf.Variable(tf.random_normal([n_nodes_hl1]))}
	hidden_2 = {'weights' : tf.Variable(tf.random_normal([n_nodes_hl1, n_nodes_hl2])), 'biases' : tf.Variable(tf.random_normal([n_nodes_hl2]))}
	hidden_3 = {'weights' : tf.Variable(tf.random_normal([n_nodes_hl2, n_nodes_hl3])), 'biases' : tf.Variable(tf.random_normal([n_nodes_hl3]))}
	
	output_layer = {'weights' : tf.Variable(tf.random_normal([n_nodes_hl3, MAX_LINKS+1])), 'biases' : tf.Variable(tf.random_normal([MAX_LINKS+1]))}
	
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
	
	batcher = xy_batcher(files)
	with tf.Session() as sess:
		sess.run(tf.global_variables_initializer())

		batch = batcher.next()
		epoch_x,epoch_y = batch
		#print epoch_x[0]
		#exit()
		epoch = 1
		while epoch < 1000:
			_,c = sess.run([optimizer, cost], feed_dict = {x: epoch_x, y: epoch_y})
			print "Epoch",epoch,"completed of",N_EPOCHS,"loss:",c
			epoch += 1
		print prediction.eval(feed_dict={x:[epoch_x[0]]})
		

		"""
		for epoch in range(N_EPOCHS):
			
			batch = batcher.next()
			
			if not batch:
				break
			epoch_x,epoch_y = batch
			
			#print len(epoch_x), len(epoch_y), len(epoch_x[0]), len(epoch_y[0])
			_,c = sess.run([optimizer, cost], feed_dict = {x: epoch_x, y: epoch_y})
			print "Epoch",epoch,"completed of",N_EPOCHS,"loss:",c
		"""

		#correct = tf.equal(tf.argmax(prediction, 0), tf.argmax(y, 0))
		#accuracy = tf.reduce_mean(tf.cast(correct, 'float'))
		#print "Accuracy:",accuracy.eval({x:test_data, y:test_labels})
		#print tf.argmax(prediction,0)
		#print tf.argmax(y,0)
		#print prediction.eval(feed_dict={x:test_data})
			
			

if '__main__' == __name__:
	
	input_files = os.listdir(INPUT_FOLDER)
	print len(input_files)
	
	n_classes = len(input_files) * PERMUTATIONS # robots * variations, # of files * # of permutations
	
	x = tf.placeholder('float', [None, MAX_CENTROIDS * 3])
	y = tf.placeholder('float', [None, MAX_LINKS+1])
	
	test_labels = [_pad_label((3,3500,5000,2500)), ]
	test_data = [prepare_skeleton("#Skeleton#110,-2181,2125	776,-256,728	1452,-1181,2161	-920,-3102,2254	1999,-366,1674	272,-230,173	-322,-2666,2087	2288,-632,2110	-365,-2258,3122	579,-1719,2168	1441,-318,1257")]

	#print test_labels, test_data

	train_neural_network(input_files, x, y, test_labels, test_data)











