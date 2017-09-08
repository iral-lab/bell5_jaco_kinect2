import sys, os, random, cPickle, time, code, math, gzip, glob
import tensorflow as tf
from tensorflow.python import debug as tf_debug
from generator import HEADER_DIVIDER, SKELETON_MARKER, LENGTHS_HEADER, MAX_CENTROIDS, MAX_LINKS, PERMUTATIONS, get_skeleton_points

#code.interact(local=dict(globals(), **locals())) 

N_EPOCHS = 40
N_BATCHES = 10

DEFAULT_HIDDEN_LAYERS = 2
DEFAULT_NODES_PER_LAYER = 100

MAX_FILES_TESTING = None #100

RUNNING_ON_MAC = os.path.exists('./.on_mac')

INPUT_FOLDER = 'committed_clouds/' if RUNNING_ON_MAC else 'clouds/'

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
		print "Consider decompressing with ' zcat",COMPRESSED_DATA_CACHE," > ",DATA_CACHE,"'"
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
		# print file
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
	print round(time.time() - start,2), "Dumped to", COMPRESSED_DATA_CACHE
	cPickle.dump([data,labels], open(DATA_CACHE,'w'))
	print round(time.time() - start,2), "Dumped to", DATA_CACHE
	

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
	out_layer = None
	
	# Store layers weight & bias
	if True:
	
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
		
	else:
		# super simple one
		weights = {
			'out': tf.Variable(tf.random_normal([n_input, class_length]))
		}
		biases = {
			'out': tf.Variable(tf.random_normal([class_length]))
		}
		# Output layer with linear activation
		out_layer = tf.matmul(x, weights['out']) + biases['out']
	
	return out_layer



LINK_COUNT_WEIGHTING = 100
def get_cost(prediction, y, class_length, reduced = True):


	correct_noop = tf.transpose(tf.transpose(y))
	prediction_noop = tf.transpose(tf.transpose(prediction))

	correct_transpose = tf.transpose(y)
	prediction_transpose = tf.transpose(prediction)

	correct_link_counts = tf.cast(tf.gather(correct_transpose, 0), tf.int32)
	predicted_link_counts = tf.cast(tf.gather(prediction_transpose, 0), tf.int32)


	correct_length_mask = tf.sequence_mask(correct_link_counts, class_length - 1, tf.int32)

	shift_mask = tf.cast(tf.zeros([tf.shape(correct_transpose)[1],1]), tf.int32)

	shifted_correct_length_mask = tf.cast(tf.concat([shift_mask, correct_length_mask], 1), tf.float32)
	reshaped_shifted_correct_length_mask = tf.reshape(shifted_correct_length_mask, [-1])

	reshaped_correct = tf.reshape(y, [-1])
	reshaped_prediction = tf.reshape(prediction, [-1])

	correct_link_lengths = tf.reshape(reshaped_shifted_correct_length_mask * reshaped_correct, tf.shape(Y))
	predicted_link_lengths = tf.reshape(reshaped_shifted_correct_length_mask * reshaped_prediction, tf.shape(Y))

	l2_distance = tf.sqrt( tf.reduce_sum(tf.square(tf.subtract(correct_link_lengths, predicted_link_lengths)), reduction_indices=1))

	wrong_link_counts = tf.cast(LINK_COUNT_WEIGHTING * tf.abs(correct_link_counts - predicted_link_counts), tf.float32)

	penalty = l2_distance + wrong_link_counts
	
	penalty_sum = tf.reduce_sum(penalty)

	return penalty_sum if reduced else penalty
	
def get_accuracy(prediction, y, class_length):
	penalties = get_cost(prediction, y, class_length, False)
	considered_correct = 100 #LINK_COUNT_WEIGHTING * 0.001
	accurate_predictions = tf.cast(tf.less(penalties, considered_correct), tf.int32)
	accuracy_ratio = tf.cast(tf.count_nonzero(accurate_predictions), tf.float32) / tf.cast(tf.shape(y)[0], tf.float32)
	
	return accurate_predictions
	
	

def init_weights(shape):
	return tf.Variable(tf.random_normal(shape, stddev=0.01))

if '__main__' == __name__:

	
	if '-h' in sys.argv or '--help' in sys.argv:
		print "Usage: python", sys.argv[0]
		print "Usage: python", sys.argv[0],"num_hidden_layers num_nodes_per_layer"
		print "Default: python", sys.argv[0], DEFAULT_HIDDEN_LAYERS, DEFAULT_NODES_PER_LAYER
		exit()


	if not os.path.exists(COMPRESSED_DATA_CACHE):
		print "Generating data cache from", INPUT_FOLDER
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

	# cost = tf.reduce_mean(tf.cast(tf.size(Y), "float") - tf.cast(tf.size(pred), "float"))
	# cost = tf.py_func(custom_cost_function, [pred, Y], [tf.float32])
	# cost = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(logits=pred, labels=Y))
	cost = get_cost(pred, Y, class_length)
	optimizer = tf.train.AdamOptimizer(learning_rate=learning_rate).minimize(cost)
	
	print "Trainable:", tf.trainable_variables()
	
	cost_stats = []
	accuracy_stats = []

	overall_start = time.time()


	with tf.Session() as sess:
		# sess = tf_debug.LocalCLIDebugWrapperSession(sess)
		# you need to initialize all variables
		tf.global_variables_initializer().run()
		
		# correct_prediction = tf.equal(tf.argmax(pred, 1), tf.argmax(Y, 1))
		# accuracy = tf.reduce_mean(tf.cast(correct_prediction, "float"))
		accuracy = get_accuracy(pred, Y, class_length)
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
				# print accuracy_val.tolist()
				lst = accuracy_val.tolist()
				if 1 in lst:
					index = lst.index(1)
					print index
					print test_batch[0][index]
					print test_batch[1][index]
					print pred.eval(feed_dict = {X:[test_batch[0][index]]}).tolist()
					print accuracy.eval({X: [test_batch[0][index]], Y: [test_batch[1][index]]})
					print sess.run(cost, feed_dict = {X:[test_batch[0][index]], Y:[test_batch[1][index]]})
					
					
					#print sess.run(cost, feed_dict = {X:[[-2821, 4738, 744, -3113, 1327, -1298, -1438, 5478, -2501, -2012, -743, -760, -2890, 2115, -1505, -3247, 356, 727, -1942, 5654, -799, -2177, 3839, -1989, -264, -245, -278, -3329, -642, -288, -1004, 6470, -2854, -2457, 5205, 13, -1437, 6330, -2332, -1133, -459, -554, -1844, 4634, -2280, -3142, 4265, 1481, -2767, -1004, -983, -1634, 6073, -1474, -2505, 2915, -1758, -3466, 508, -403, -3229, -70, 169, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]], Y: [[5, 3500, 2500, 2000, 7000, 6000, 0.0, 0.0, 0.0]], Y:[[5, 3500, 2500, 2000, 7000, 6000, 0.0, 0.0, 0.0]]})
					
					#print accuracy.eval({X: [[-2821, 4738, 744, -3113, 1327, -1298, -1438, 5478, -2501, -2012, -743, -760, -2890, 2115, -1505, -3247, 356, 727, -1942, 5654, -799, -2177, 3839, -1989, -264, -245, -278, -3329, -642, -288, -1004, 6470, -2854, -2457, 5205, 13, -1437, 6330, -2332, -1133, -459, -554, -1844, 4634, -2280, -3142, 4265, 1481, -2767, -1004, -983, -1634, 6073, -1474, -2505, 2915, -1758, -3466, 508, -403, -3229, -70, 169, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]], Y: [[5, 3500, 2500, 2000, 7000, 6000, 0.0, 0.0, 0.0]]})
					print
					print
					print

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










