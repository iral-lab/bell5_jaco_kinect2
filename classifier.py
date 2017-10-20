import sys, os, random, cPickle, time, code, math, gzip, glob, popen2
import tensorflow as tf
from tensorflow.contrib import rnn
from tensorflow.python import debug as tf_debug
from generator import HEADER_DIVIDER, SKELETON_MARKER, LENGTHS_HEADER, MAX_CENTROIDS, MAX_LINKS, PERMUTATIONS, get_skeleton_points

#code.interact(local=dict(globals(), **locals())) 

RUN_MLP, RUN_RNN = range(2)
RUN_TYPE = RUN_RNN

MLP_TAG = "MLP"
RNN_TAG = "RNN"

N_EPOCHS = 50
N_BATCHES = 10

DEFAULT_HIDDEN_LAYERS = 3
DEFAULT_NODES_PER_LAYER = 100

MAX_FILES_TESTING = None #100

RUNNING_ON_MAC = os.path.exists('./.on_mac')
RUNNING_ON_AWS = os.path.exists('./.on_aws')

INPUT_FOLDER = 'committed_clouds/' if RUNNING_ON_MAC else 'clouds/'

DATA_CACHE = '_classifier_input_'+(MLP_TAG if RUN_TYPE == RUN_MLP else RNN_TAG)+'.pickle'
COMPRESSED_DATA_CACHE = DATA_CACHE+".gz"


S3_DESTINATION = None
S3_FOLDER = None
if RUN_TYPE == RUN_RNN:
	S3_FOLDER = "umbc.research/robot_learn_classifier_rnn/"
elif RUN_TYPE == RUN_MLP:
	S3_FOLDER = "umbc.research/robot_learn_classifier/"

if RUNNING_ON_AWS and not S3_FOLDER:
	print "Could not set AWS S3 location"
	exit()
S3_DESTINATION = "s3://" + S3_FOLDER

def run_cmd(cmd):
	o,i = popen2.popen2(cmd)
	return o.read()


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
def data_label_batcher(data_cache, label_cache):
	global BATCH_CACHE
	
	if not BATCH_CACHE or len(BATCH_CACHE) == 0:
		BATCH_CACHE = []

		so_far = 0
		size = len(data_cache)

		batch_size = int(math.ceil(1.0 * size / N_BATCHES));

		batches = []

		print "Creating",N_BATCHES,"batches of",batch_size,"items, total of",size
		batch_i = 0
		while so_far < size:
			data = data_cache[so_far:so_far + batch_size]
			labels = label_cache[so_far:so_far + batch_size]

			batch = (data,labels)
			BATCH_CACHE.append( batch )
			
			batch_i += 1
			so_far += batch_size
	
		# prune out bad data based on how many permutations we expect
		for i,batch in enumerate(BATCH_CACHE):
			invalid_indexes = [index for index,records in enumerate(batch[0]) if len(records) <> PERMUTATIONS]
			if len(invalid_indexes) > 0:
				new_data = [record for index,record in enumerate(batch[0]) if not index in invalid_indexes]
				new_labels = [record for index,record in enumerate(batch[1]) if not index in invalid_indexes]
			
				BATCH_CACHE[i] = [new_data, new_labels]
	
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
		print "Loading from decompressed data cache:",load_from
		handle = open(load_from, 'r')

	start = time.time()
	data_cache,label_cache = cPickle.load(handle)
	#code.interact(local=dict(globals(), **locals()))
	handle.close()
	print "loaded", len(data_cache), "pairs in", int(time.time() - start), "seconds"

	return (data_cache, label_cache)

def gen_datacache(files, from_AWS = False):
	start = time.time()
	data, labels = get_data_labels(files, from_AWS)
	
	if data and labels:
		# code.interact(local=dict(globals(), **locals())) 
		print round(time.time() - start,2), "Saving",len(data),"pairs"
		cPickle.dump([data,labels], gzip.GzipFile(COMPRESSED_DATA_CACHE,'w'))
		print round(time.time() - start,2), "Dumped to", COMPRESSED_DATA_CACHE
		cPickle.dump([data,labels], open(DATA_CACHE,'w'))
		print round(time.time() - start,2), "Dumped to", DATA_CACHE
		if RUNNING_ON_AWS:
			print "Uploading to s3"
			run_cmd("aws s3 --region us-east-1 cp "+COMPRESSED_DATA_CACHE+" "+S3_DESTINATION)
			run_cmd("aws s3 --region us-east-1 cp "+DATA_CACHE+" "+S3_DESTINATION)
	else:
		print "No data or no labels returned"
		exit()
	

def _read_files(files, from_AWS):
	start = time.time()
	num_files = len(files)
	
	for i, file in enumerate(files):
		if file in [".DS_Store", ""]:
			continue
		
		if from_AWS:
			cmd = "aws s3 cp " + S3_DESTINATION + "clouds/"+file+" clouds/"+file
			print num_files,i,">",cmd
			run_cmd(cmd)
		
		if i % 1000 == 0:
			print round(time.time() - start,2), i, round(100.0 * i / len(files),2),"%",file
		yield file
		
		
		if from_AWS:
			run_cmd("rm clouds/"+file)
		
		if MAX_FILES_TESTING and i >= MAX_FILES_TESTING:
			break

def get_data_labels(files, from_AWS = False):
	files = sorted(files)
	data = []
	labels = []
	all_pairs = []
	# files = random.shuffle(files)
	
	for file in _read_files(files, from_AWS):
		all_pairs += [_ for _ in frame_reader(file)]
		#code.interact(local=dict(globals(), **locals())) 
	
	random.shuffle(all_pairs)
	
	data += [pair[0] for pair in all_pairs]
	labels += [pair[1] for pair in all_pairs]
	
	return [data, labels]	

def frame_reader(file):
	with gzip.GzipFile(INPUT_FOLDER+file, 'r') as handle:
		skeleton_frame = None
		label = None
		skeleton_frames = []
		input = handle.readline()
		while input:
			line = input.strip()
			input = handle.readline()
			
			if "" == line:
				continue
			
			if RUN_TYPE == RUN_MLP and HEADER_DIVIDER == line and skeleton_frame:
				if skeleton_frame and label:
					yield (skeleton_frame, label)
				else:
					print "Missing skeleton_frame or label"
				skeleton_frame = None
				label = None
				continue
			
			if SKELETON_MARKER in line:
				skeleton_frame = prepare_skeleton(line)
				if RUN_TYPE == RUN_RNN:
					skeleton_frames.append(skeleton_frame)
				continue
			
			if LENGTHS_HEADER in line:
				label = get_label(line)
				continue

		if RUN_TYPE == RUN_MLP and skeleton_frame and label:
			yield (skeleton_frame, label)
		elif RUN_TYPE == RUN_RNN:
			yield (skeleton_frames, label)


def mlp_model(x, n_input, class_length, hidden_layers, nodes_per_layer):
	if hidden_layers < 1:
		print "Hidden_layers must be at least 1"
		exit()
	
	# print n_input, class_length, hidden_layers, nodes_per_layer
	weights = {}
	biases = {}
	
	for i in range(hidden_layers + 1):
		if i == 0:
			# first layer
			weights[i] = tf.Variable(tf.random_normal([ n_input, nodes_per_layer ]))			
			biases[i] = tf.Variable(tf.random_normal([nodes_per_layer])),
		elif i == hidden_layers:
			# out layer
			weights[i] = tf.Variable(tf.random_normal([ nodes_per_layer, class_length ]))
			biases[i] = tf.Variable(tf.random_normal([class_length])),
		else:
			# internal layers
			weights[i] = tf.Variable(tf.random_normal([ nodes_per_layer, nodes_per_layer ]))
			biases[i] = tf.Variable(tf.random_normal([nodes_per_layer])),
	
	network = tf.nn.relu(tf.add(tf.matmul(x, weights[0]), biases[0]))
	for i in range(1, hidden_layers + 1):
		network = tf.nn.relu(tf.add(tf.matmul(network, weights[i]), biases[i]))
	return network

def rnn_model(x, n_input, class_length, hidden_layers, nodes_per_layer):
	# RNN output node weights and biases
	weights = {
	    'out': tf.Variable(tf.random_normal([nodes_per_layer, class_length]))
	}
	biases = {
	    'out': tf.Variable(tf.random_normal([class_length]))
	}
	#init_state = tf.zeros([n_hidden, frame_length])
	
	cells = [tf.contrib.rnn.LSTMCell(nodes_per_layer) for _ in xrange(hidden_layers)]
	multi_rnn_cell = tf.contrib.rnn.MultiRNNCell(cells)
	print
	print	
	print "n_input",n_input
	print "class_length",class_length
	print "hidden_layers",hidden_layers
	print "nodes_per_layer",nodes_per_layer
	print "X:",x
	print "single rnn cells:",cells
	print "multi_rnn_cell:",multi_rnn_cell
		
	outputs, states = tf.nn.dynamic_rnn(multi_rnn_cell, x, dtype = tf.float32)
	print "outputs:",outputs
	print "states:",states
	
	pred = tf.nn.relu(tf.add(tf.matmul(outputs[-1], weights['out']), biases['out']))
	print "pred",pred
	print
	print
	
	return pred
	
	
	

LINK_COUNT_WEIGHTING = 100
def get_mlp_cost(prediction, y, class_length, reduced = True):


	correct_noop = tf.transpose(tf.transpose(y))
	prediction_noop = tf.transpose(tf.transpose(prediction))

	correct_transpose = tf.transpose(y)
	prediction_transpose = tf.transpose(prediction)

	correct_link_counts = tf.cast(tf.gather(correct_transpose, 0), tf.int32)
	predicted_link_counts = tf.cast(tf.gather(prediction_transpose, 0), tf.int32)


	#correct_length_mask = tf.sequence_mask(correct_link_counts, class_length - 1, tf.int32)
	# This mask below doesn't mask away the extra values, only will zero out the link-counts.
	correct_length_mask = tf.cast(tf.ones([tf.shape(correct_transpose)[1], tf.shape(correct_transpose)[0]-1]), tf.int32)

	shift_mask = tf.cast(tf.zeros([tf.shape(correct_transpose)[1],1]), tf.int32)

	shifted_correct_length_mask = tf.cast(tf.concat([shift_mask, correct_length_mask], 1), tf.float32)
	reshaped_shifted_correct_length_mask = tf.reshape(shifted_correct_length_mask, [-1])

	reshaped_correct = tf.reshape(y, [-1])
	reshaped_prediction = tf.reshape(prediction, [-1])

	correct_link_lengths = tf.reshape(reshaped_shifted_correct_length_mask * reshaped_correct, tf.shape(y))
	predicted_link_lengths = tf.reshape(reshaped_shifted_correct_length_mask * reshaped_prediction, tf.shape(y))

	adjusted_correct_link_lengths = correct_link_lengths / 10000
	adjusted_predicted_link_lengths = predicted_link_lengths / 10000

	l2_distance = tf.sqrt( tf.reduce_sum(tf.square(tf.subtract(adjusted_correct_link_lengths, adjusted_predicted_link_lengths)), reduction_indices=1))
	
	abs_diff_counts = tf.abs(correct_link_counts - predicted_link_counts)
	
	# remove the link-count penalty
	# wrong_link_counts = tf.cast(LINK_COUNT_WEIGHTING, tf.float32) * tf.cast(abs_diff_counts, tf.float32)
	penalty = l2_distance #+ wrong_link_counts
	
	penalty_sum = tf.reduce_sum(penalty)

	return penalty_sum if reduced else penalty

def get_rnn_cost(pred, y):
	print "pred:",pred
	print "Y:",y
	l2_distance = tf.sqrt( tf.reduce_sum(tf.square(tf.subtract(pred, y)), reduction_indices=1))
	penalty_sum = tf.reduce_sum(l2_distance)
	return penalty_sum

def run_batch(X, Y, sess, batch, cost, optimizer = None):
	batch_cost = 0
	
	to_run = [cost]
	if optimizer:
		to_run.append(optimizer)
	
	if RUN_TYPE == RUN_MLP:
		epoch_x,epoch_y = batch
		results = sess.run(to_run, feed_dict = {X: epoch_x, Y: epoch_y})
		
		batch_cost = results[0]
	
	elif RUN_TYPE == RUN_RNN:
		data, labels = batch
		for i in xrange(len(data)):
			epoch_x = [data[i]]
			epoch_y = [labels[i]]
			results = sess.run(to_run, feed_dict = {X: epoch_x, Y: epoch_y})
			batch_cost += results[0]
	
	return batch_cost

def run_test(data_cache, label_cache, hidden_layers, nodes_per_layer):
	
	class_length = MAX_LINKS + 1
	n_input = MAX_CENTROIDS * 3

	learning_rate = 0.001
	
	
	# code.interact(local=dict(globals(), **locals())) 
	# print "Class length:",class_length
	

	X = None
	Y = None
	pred = None
	cost = None
	if RUN_TYPE == RUN_MLP:
		X = tf.placeholder('float', [None, n_input])
		Y = tf.placeholder('float', [None, class_length])
		pred = mlp_model(X, n_input, class_length, hidden_layers, nodes_per_layer)
		cost = get_mlp_cost(pred, Y, class_length)
	
	elif RUN_TYPE == RUN_RNN:
		X = tf.placeholder('float', [None, PERMUTATIONS, n_input])
		Y = tf.placeholder('float', [None, class_length])
		pred = rnn_model(X, n_input, class_length, hidden_layers, nodes_per_layer)
		cost = get_rnn_cost(pred, Y)
	
	optimizer = tf.train.AdamOptimizer(learning_rate=learning_rate).minimize(cost)
	
	# print "Trainable:", tf.trainable_variables()
	
	cost_stats = []

	overall_start = time.time()


	with tf.Session() as sess:
		# sess = tf_debug.LocalCLIDebugWrapperSession(sess)
		# you need to initialize all variables
		tf.global_variables_initializer().run()
		
		# correct_prediction = tf.equal(tf.argmax(pred, 1), tf.argmax(Y, 1))
		# accuracy = tf.reduce_mean(tf.cast(correct_prediction, "float"))
		# accuracy = get_accuracy(pred, Y, class_length)
		for i in range(N_EPOCHS):
			epoch_start = time.time()
			test_batch = None
			batcher = data_label_batcher(data_cache, label_cache)
			
			epoch_cost = 0
			num_batches = 0
			
			for j,batch in enumerate(batcher):
				if j == 0:
					test_batch = batch
					continue
				
				epoch_cost += run_batch(X, Y, sess, batch, cost, optimizer)
				num_batches += 1
				# print i,j,epoch_cost
			avg_train_cost = round(1.0 * epoch_cost / num_batches, 2) if num_batches > 0 else 0.0
			
			test_cost = run_batch(X, Y, sess, test_batch, cost)
			
			print ">", round(time.time() - overall_start,2), round(time.time() - epoch_start,2), i, j, "Avg epoch train cost:", avg_train_cost , "Test cost:", test_cost
			
			cost_stats.append( "\t".join([str(x) for x in [i, avg_train_cost, test_cost, 1/test_cost]]))

	stats_folder = "run_stats/run_"+str(int(time.time()))+"_"+str(hidden_layers)+"_"+str(nodes_per_layer)+"/"
	os.makedirs(stats_folder)

	with open(stats_folder+'tf_results.csv', 'w') as handle:
		handle.write("\t".join(['Epoch', 'Batch', 'Train cost', 'Test cost', '1/Test cost'])+"\n")
		handle.write("\n".join(cost_stats)+"\n")
	
	if RUNNING_ON_AWS:
		cmd = "aws s3 --region us-east-1 cp " + stats_folder + "* " + S3_DESTINATION + stats_folder
		print cmd
		run_cmd(cmd)
		run_cmd("rm -rf "+stats_folder)

def run_hyper(data_cache, label_cache):
	layer_range = range(1,10)
	nodes_per_layer_range = range(20, 200, 10)
	
	for layers in layer_range:
		for nodes_per_layer in nodes_per_layer_range:
			print "HYPER: ", layers, nodes_per_layer
			try:
				run_test(data_cache, label_cache, layers, nodes_per_layer)
			except:
				print "Caught error, continuing"
				pass


if '__main__' == __name__:
	
	if '-h' in sys.argv or '--help' in sys.argv:
		print "Usage: python", sys.argv[0]
		print "Usage: python", sys.argv[0],"num_hidden_layers num_nodes_per_layer"
		print "Default: python", sys.argv[0], DEFAULT_HIDDEN_LAYERS, DEFAULT_NODES_PER_LAYER
		exit()

	
	if not os.path.exists(COMPRESSED_DATA_CACHE):
		should_generate = True
		if RUNNING_ON_AWS:
			compressed_available = not "" == run_cmd("aws s3 --region us-east-1 ls " + S3_DESTINATION + COMPRESSED_DATA_CACHE)
			uncompressed_available = not "" == run_cmd("aws s3 --region us-east-1 ls " + S3_DESTINATION + DATA_CACHE)
			
			if compressed_available:
				run_cmd("aws s3 --region us-east-1 cp " + S3_DESTINATION + COMPRESSED_DATA_CACHE + " .")
			if uncompressed_available:
				run_cmd("aws s3 --region us-east-1 cp " + S3_DESTINATION + DATA_CACHE + " .")
			
			if compressed_available or uncompressed_available:
				print "Successfully downloaded from s3"
				should_generate = False
			else:
				print "Could not download from s3"
		
		if should_generate and RUNNING_ON_AWS:
			print "Generating data cache from AWS"
			input_files = run_cmd("aws s3 ls " + S3_FOLDER + "/clouds/ | ruby -e \"STDIN.readlines.each{|x| puts x.split.join(' ')}\" | cut -d' ' -f4").split("\n")
			gen_datacache(input_files, True)
		elif should_generate:
			print "Generating data cache from", INPUT_FOLDER
			input_files = os.listdir(INPUT_FOLDER)
			gen_datacache(input_files)
		exit()
	
	hidden_layers = DEFAULT_HIDDEN_LAYERS
	nodes_per_layer = DEFAULT_NODES_PER_LAYER

	data_cache, label_cache = load_data_cache()
	# code.interact(local=dict(globals(), **locals()))
	if '--hyper' in sys.argv:
		run_hyper(data_cache, label_cache)
		sys.exit()
	
	elif len(sys.argv) == 3:
		hidden_layers = int(sys.argv[1])
		nodes_per_layer = int(sys.argv[2])

	print "Running with", hidden_layers, "layers, each with", nodes_per_layer, "nodes"
	
	run_test(data_cache, label_cache, hidden_layers, nodes_per_layer)
	













