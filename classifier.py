import sys, os, random, cPickle, time, code, math
import tensorflow as tf
from generator import HEADER_DIVIDER, SKELETON_MARKER, LENGTHS_HEADER, MAX_CENTROIDS, MAX_LINKS, PERMUTATIONS, get_skeleton_points

#code.interact(local=dict(globals(), **locals())) 

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
ALREADY_MADE_BATCHES = None
def naive_batcher(label_lookup = None):
	if not os.path.exists(DATA_CACHE):
		print "No data cache, please generate first"
		exit()
	global ACTUAL_DATA_CACHE
	global ALREADY_MADE_BATCHES

	if not ALREADY_MADE_BATCHES:

		if not label_lookup:
			label_lookup = get_label_lookup()
		
		so_far = 0
		size = len(ACTUAL_DATA_CACHE[0])

		batch_size = int(math.ceil(1.0 * size / N_BATCHES));

		print "Creating",N_BATCHES,"of",batch_size,"items, total of",size

		ALREADY_MADE_BATCHES = []
		while so_far < size:
			data = ACTUAL_DATA_CACHE[0][so_far:so_far + batch_size]
			labels = ACTUAL_DATA_CACHE[1][so_far:so_far + batch_size]

			for i,label in enumerate(labels):
				labels[i] = [0] * len(label_lookup)
				labels[i][label_lookup[tuple(label)]] = 1

			ALREADY_MADE_BATCHES.append((data, labels))
			so_far += batch_size

	for data,labels in ALREADY_MADE_BATCHES:
		yield (data, labels)

def get_label_lookup():
	if not os.path.exists(DATA_CACHE):
		print "No data cache, please generate first"
		exit()
	global ACTUAL_DATA_CACHE

	if not ACTUAL_DATA_CACHE:
		print "loading"
		start = time.time()
		ACTUAL_DATA_CACHE = cPickle.load(open(DATA_CACHE,'r'))
		print "loaded",len(ACTUAL_DATA_CACHE[0]),"pairs in",int(time.time() - start),"seconds"

	lookup = {}
	labels_so_far = 0
	for label_i,label in enumerate(ACTUAL_DATA_CACHE[1]):
		label = tuple(label)
		if not label in lookup:
			lookup[label] = labels_so_far
			labels_so_far += 1
	return lookup

def gen_datacache(files):
	files = sorted(files)
	data = []
	labels = []
	for i,file in enumerate(files):
		if i % 100 == 0:
			print i,round(100.0 * i / len(files),2),"%"
		pairs = [_ for _ in naive_frame_reader(file)]
		data += [pair[0] for pair in pairs]
		labels += [pair[1] for pair in pairs]

	print "Saving",len(data),"pairs"
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


def mlp_model(x, n_input, num_classes):
	n_hidden_1 = n_hidden_2 = 100
	# Store layers weight & bias
	weights = {
		'h1': tf.Variable(tf.random_normal([n_input, n_hidden_1])),
		'h2': tf.Variable(tf.random_normal([n_hidden_1, n_hidden_2])),
		'out': tf.Variable(tf.random_normal([n_hidden_2, num_classes]))
	}
	biases = {
		'b1': tf.Variable(tf.random_normal([n_hidden_1])),
		'b2': tf.Variable(tf.random_normal([n_hidden_2])),
		'out': tf.Variable(tf.random_normal([num_classes]))
	}
	# Hidden layer with RELU activation
	layer_1 = tf.add(tf.matmul(x, weights['h1']), biases['b1'])
	layer_1 = tf.nn.relu(layer_1)
	# Hidden layer with RELU activation
	layer_2 = tf.add(tf.matmul(layer_1, weights['h2']), biases['b2'])
	layer_2 = tf.nn.relu(layer_2)
	# Output layer with linear activation
	out_layer = tf.matmul(layer_2, weights['out']) + biases['out']
	return out_layer

	

def init_weights(shape):
	return tf.Variable(tf.random_normal(shape, stddev=0.01))

if '__main__' == __name__:

	if not os.path.exists(DATA_CACHE):	
		input_files = os.listdir(INPUT_FOLDER)
		gen_datacache(input_files)
		exit()
	


	class_length = MAX_LINKS+1
	n_input = MAX_CENTROIDS * 3

	learning_rate = 0.001

	label_lookup = get_label_lookup()
	num_classes = len(label_lookup)
	print "Classes:",num_classes

	X = tf.placeholder('float', [None, n_input])
	Y = tf.placeholder('float', [None, num_classes])

	pred = mlp_model(X, n_input, num_classes)
	cost = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(logits=pred, labels=Y))
	optimizer = tf.train.AdamOptimizer(learning_rate=learning_rate).minimize(cost)

	cost_stats = []
	accuracy_stats = []

	with tf.Session() as sess:
		# you need to initialize all variables
		tf.global_variables_initializer().run()
		first_batch = None
		correct_prediction = tf.equal(tf.argmax(pred, 1), tf.argmax(Y, 1))
		accuracy = tf.reduce_mean(tf.cast(correct_prediction, "float"))
		for i in range(N_EPOCHS):
			
			for j,batch in enumerate(naive_batcher(label_lookup)):
				
				if not first_batch:
					first_batch = batch
					continue

				while len(cost_stats) <= j:
					cost_stats.append( [] )
					accuracy_stats.append( [] )

				epoch_x,epoch_y = batch

				_,c = sess.run([optimizer, cost], feed_dict={X:epoch_x, Y: epoch_y})
				accuracy_val = accuracy.eval({X: first_batch[0], Y: first_batch[1]})
				print ">", i, j, "Cost:", c, "Accuracy:", accuracy_val

				cost_stats[j].append(c)
				accuracy_stats[j].append(accuracy_val)

	with open('tf_cost_'+str(int(time.time()))+".csv",'w') as handle:
		to_write = [",".join([str(x) for x in line]) for line in cost_stats]
		handle.write("\n".join(to_write)+"\n")

	with open('tf_accuracy_'+str(int(time.time()))+".csv",'w') as handle:
		to_write = [",".join([str(x) for x in line]) for line in accuracy_stats]
		handle.write("\n".join(to_write)+"\n")










