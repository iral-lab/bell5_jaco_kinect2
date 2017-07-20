import sys, os, random, cPickle
import tensorflow as tf
from generator import HEADER_DIVIDER, SKELETON_MARKER, LENGTHS_HEADER, MAX_CENTROIDS, MAX_LINKS, PERMUTATIONS, get_skeleton_points

N_EPOCHS = 10
N_BATCHES = 10

INPUT_FOLDER = 'clouds/'

DATA_CACHE = '_classifier_input.pickle'

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


ACTUAL_DATA_CACHE = None
def naive_batcher():
	if not os.path.exists(DATA_CACHE):
		print "No data cache, please generate first"
		exit()
	global ACTUAL_DATA_CACHE

	if not ACTUAL_DATA_CACHE:
		print "loading"
		ACTUAL_DATA_CACHE = cPickle.load(open(DATA_CACHE,'r'))
		print "loaded"

	batch_size = 1
	so_far = 0
	size = len(ACTUAL_DATA_CACHE[0])

	while so_far < size:
		yield (ACTUAL_DATA_CACHE[0][so_far:so_far + batch_size], ACTUAL_DATA_CACHE[1][so_far:so_far+batch_size])
		so_far += batch_size


def gen_datacache(files):
	files = sorted(files)[:100]
	data = []
	labels = []
	for i,file in enumerate(files):
		if i % 100 == 0:
			print i,round(100.0 * i / len(files),2),"%"
		pairs = [_ for _ in naive_frame_reader(file)]
		data += [pair[0] for pair in pairs]
		labels += [pair[1] for pair in pairs]

	print "Saving"
	cPickle.dump([data,labels], open(DATA_CACHE,'w'))
	print "Dumped"
		

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
					yield (skeleton_frame, label)
				else:
					print "Missing skeleton_frame or label"
				skeleton_frame = None
				label = None
				continue
			
			if SKELETON_MARKER in line:
				skeleton_frame = prepare_skeleton(line)
				continue
			
			if LENGTHS_HEADER in line:
				label = get_label(line)
				continue

		if skeleton_frame and label:
			yield (skeleton_frame, label)


def mlp_model(x, w_h, w_o):
	h = tf.nn.sigmoid(tf.matmul(x, w_h))
	return tf.matmul(h, w_o)
	

def init_weights(shape):
	return tf.Variable(tf.random_normal(shape, stddev=0.01))


if '__main__' == __name__:

	if not os.path.exists(DATA_CACHE):	
		input_files = os.listdir(INPUT_FOLDER)
		gen_datacache(input_files)
		exit()
	

	class_length = MAX_LINKS+1

	X = tf.placeholder('float', [None, MAX_CENTROIDS * 3])
	Y = tf.placeholder('float', [None, class_length])

	hidden_layer_nodes = MAX_CENTROIDS * 3

	w_h = init_weights([MAX_CENTROIDS * 3, hidden_layer_nodes])
	w_o = init_weights([hidden_layer_nodes, class_length])

	py_x = mlp_model(X, w_h, w_o)

	cost = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(logits=py_x, labels=Y))
	train_op = tf.train.GradientDescentOptimizer(0.05).minimize(cost)
	predict_op = tf.argmax(py_x, 1)

	with tf.Session() as sess:
		# you need to initialize all variables
		tf.global_variables_initializer().run()

		for i in range(N_EPOCHS):
			first_batch = None
			for j,batch in enumerate(naive_batcher()):
				if not first_batch:
					first_batch = batch
				epoch_x,epoch_y = batch

				sess.run(train_op, feed_dict={X:epoch_x, Y: epoch_y})
				guess = sess.run(predict_op, feed_dict = {X: epoch_x, Y:epoch_y})
				print ">", i, j, epoch_y[:5], len(guess), guess
				# print ">",j
			if first_batch:
				print "_", i, j, sess.run(predict_op, feed_dict = {X: first_batch[0], Y:first_batch[1]})

		# print i, np.mean(np.argmax(teY, axis=1) == sess.run(predict_op, feed_dict={X: teX}))









