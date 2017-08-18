import sys, os, random, cPickle, time, code, math, gzip, glob
import tensorflow as tf
from generator import HEADER_DIVIDER, SKELETON_MARKER, LENGTHS_HEADER, MAX_CENTROIDS, MAX_LINKS, PERMUTATIONS, get_skeleton_points

#code.interact(local=dict(globals(), **locals())) 

N_EPOCHS = 40
N_BATCHES = 10

MAX_FILES_TESTING = 100 # 

INPUT_FOLDER = 'clouds/'

SHORT_DATA_CACHE = '_classifier_short_input.pickle'
DATA_CACHE = '_classifier_input.pickle'
BATCH_CACHE_STEM = '_classifier_batches.out_'

COMPRESSED_DATA_CACHE = DATA_CACHE+".gz"

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


BATCH_CACHE = None
def naive_batcher(data_cache, label_cache):
	global BATCH_CACHE

	use_cache = True

	if not BATCH_CACHE or len(BATCH_CACHE) == 0:
		BATCH_CACHE = []

		so_far = 0
		size = len(data_cache)

		batch_size = int(math.ceil(1.0 * size / N_BATCHES));

		batches = []

		print "Creating",N_BATCHES,"of",batch_size,"items, total of",size
		batch_i = 0
		while so_far < size:
			data = data_cache[so_far:so_far + batch_size]
			labels = label_cache[so_far:so_far + batch_size]

			batch = (data,labels)
			if use_cache:
				BATCH_CACHE.append( batch )
			else:
				yield batch

			batch_i += 1
			so_far += batch_size
	if use_cache and BATCH_CACHE:
		for batch in BATCH_CACHE:
			yield batch


def load_data_cache():
	if not os.path.exists(DATA_CACHE) and not os.path.exists(COMPRESSED_DATA_CACHE):
		print "No data cache, please generate first"
		exit()

	load_from = DATA_CACHE
	if not os.path.exists(COMPRESSED_DATA_CACHE) and not os.path.exists(DATA_CACHE):
		print "No cache file available. Please generate it first"
		exit()

	handle = None
	if not os.path.exists(DATA_CACHE):
		print "Loading from compressed data cache, will take longer."
		print "Consider decompressing with 'zcat",COMPRESSED_DATA_CACHE," > ",DATA_CACHE,"'"
		load_from = COMPRESSED_DATA_CACHE
		handle = gzip.GzipFile(load_from, 'r')
	else:
		print "Loading from decompressed data cache"
		handle = open(load_from, 'r')

	start = time.time()
	data_cache,label_cache = cPickle.load(handle)
	handle.close()
	print "loaded", len(data_cache), "pairs in", int(time.time() - start), "seconds"

	return (data_cache, label_cache)

def gen_naive_datacache(files):
	files = sorted(files)
	data = []
	labels = []
	all_pairs = []
	start = time.time()
	# files = random.shuffle(files)

	for i,file in enumerate(files):
		print file
		if i % 1000 == 0:
			print round(time.time() - start,2), i, round(100.0 * i / len(files),2),"%",file
		all_pairs += [_ for _ in naive_frame_reader(file)]
		if MAX_FILES_TESTING and i >= MAX_FILES_TESTING:
			break
	random.shuffle(all_pairs)

	data += [pair[0] for pair in all_pairs]
	labels += [pair[1] for pair in all_pairs]

	# code.interact(local=dict(globals(), **locals())) 
	print round(time.time() - start,2), "Saving",len(data),"pairs"
	cPickle.dump([data,labels], gzip.GzipFile(COMPRESSED_DATA_CACHE,'w'))
	print round(time.time() - start,2), "Dumped"
		

def naive_frame_reader(file):
	with gzip.GzipFile(INPUT_FOLDER+file, 'r') as handle:
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


def mlp_model(x, n_input, class_length, hidden_layers, nodes_per_layer):
	n_hidden_1 = n_hidden_2 = nodes_per_layer
	# Store layers weight & bias
	weights = {
		'h1': tf.Variable(tf.random_normal([n_input, n_hidden_1])),
		'h2': tf.Variable(tf.random_normal([n_hidden_1, n_hidden_2])),
		'out': tf.Variable(tf.random_normal([n_hidden_2, class_length]))
	}
	biases = {
		'b1': tf.Variable(tf.random_normal([n_hidden_1])),
		'b2': tf.Variable(tf.random_normal([n_hidden_2])),
		'out': tf.Variable(tf.random_normal([class_length]))
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


def custom_cost_function(input):
	print input
	return [0.1]


def init_weights(shape):
	return tf.Variable(tf.random_normal(shape, stddev=0.01))

if '__main__' == __name__:

	DEFAULT_HIDDEN_LAYERS = 2
	DEFAULT_NODES_PER_LAYER = 100

	if '-h' in sys.argv or '--help' in sys.argv:
		print "Usage: python", sys.argv[0]
		print "Usage: python", sys.argv[0],"num_hidden_layers num_nodes_per_layer"
		print "Default: python", sys.argv[0], DEFAULT_HIDDEN_LAYERS, DEFAULT_NODES_PER_LAYER
		exit()


	if not os.path.exists(COMPRESSED_DATA_CACHE):
		print "Generating data cache"
		input_files = os.listdir(INPUT_FOLDER)
		gen_naive_datacache(input_files)
		exit()
	
	hidden_layers = DEFAULT_HIDDEN_LAYERS
	nodes_per_layer = DEFAULT_NODES_PER_LAYER

	if len(sys.argv) == 3:
		hidden_layers = int(sys.argv[1])
		nodes_per_layer = int(sys.argv[2])

	print "Running with", hidden_layers, "layers, each with", nodes_per_layer, "nodes"

	class_length = MAX_LINKS + 1
	n_input = MAX_CENTROIDS * 3

	learning_rate = 0.001
	
	data_cache, label_cache = load_data_cache()

	# code.interact(local=dict(globals(), **locals())) 
	print "Class length:",class_length
	

	X = tf.placeholder('float', [None, n_input])
	Y = tf.placeholder('float', [None, class_length])
	pred = mlp_model(X, n_input, class_length, hidden_layers, nodes_per_layer)

	#cost = tf.reduce_mean(tf.cast(tf.size(Y), "float") - tf.cast(tf.size(pred), "float"))
	# cost = tf.py_func(custom_cost_function, [pred, Y], [tf.float32])
	cost = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(logits=pred, labels=Y))
	optimizer = tf.train.AdamOptimizer(learning_rate=learning_rate).minimize(cost)
	
	print "Trainable:", tf.trainable_variables()
	
	cost_stats = []
	accuracy_stats = []

	overall_start = time.time()


	with tf.Session() as sess:
		# you need to initialize all variables
		tf.global_variables_initializer().run()
		
		correct_prediction = tf.equal(tf.argmax(pred, 1), tf.argmax(Y, 1))
		accuracy = tf.reduce_mean(tf.cast(correct_prediction, "float"))
		for i in range(N_EPOCHS):
			epoch_start = time.time()
			test_batch = None
			for j,batch in enumerate(naive_batcher(data_cache, label_cache)):
				if j == 0:
					test_batch = batch
					continue

				while len(cost_stats) <= j:
					cost_stats.append( [] )
					accuracy_stats.append( [] )

				epoch_x,epoch_y = batch
				_,c = sess.run([optimizer, cost], feed_dict={X:epoch_x, Y: epoch_y})
				accuracy_val = accuracy.eval({X: test_batch[0], Y: test_batch[1]})
				print ">", round(time.time() - overall_start,2), round(time.time() - epoch_start,2), i, j, "Cost:", c, "Accuracy:", accuracy_val
				# print test_batch[0][0]
				# print test_batch[1][0]
				# print pred.eval(feed_dict = {X:[test_batch[0][0]]})
				# code.interact(local=dict(globals(), **locals())) 

				cost_stats[j].append(c)
				accuracy_stats[j].append(accuracy_val)

	stats_folder = "run_stats/run_"+str(int(time.time()))+"_"+str(hidden_layers)+"_"+str(nodes_per_layer)+"/"
	os.makedirs(stats_folder)

	with open(stats_folder+'tf_cost.csv', 'w') as handle:
		to_write = [",".join([str(x) for x in line]) for line in cost_stats]
		handle.write("\n".join(to_write)+"\n")

	with open(stats_folder+'tf_accuracy.csv', 'w') as handle:
		to_write = [",".join([str(x) for x in line]) for line in accuracy_stats]
		handle.write("\n".join(to_write)+"\n")










