import sys, os, random, cPickle, time, code, math, gzip, glob, popen2
sys.path.append('../simulator/')
import tensorflow as tf
from tensorflow.contrib import rnn
from tensorflow.python import debug as tf_debug
from generator import HEADER_DIVIDER, SKELETON_MARKER, LENGTHS_HEADER, MAX_CENTROIDS, MAX_LINKS, MIN_LINKS, LINK_COUNTS, LABEL_SIZE, PERMUTATIONS, get_skeleton_points

#code.interact(local=dict(globals(), **locals())) 

RUN_MLP, RUN_RNN, RUN_RNN_ONE_HOT, RUN_RNN_ONE_HOT_INT = range(4)
RUN_TYPES = [RUN_MLP, RUN_RNN, RUN_RNN_ONE_HOT, RUN_RNN_ONE_HOT_INT]

RNN_VARIATIONS = [RUN_RNN, RUN_RNN_ONE_HOT, RUN_RNN_ONE_HOT_INT]

MLP_TAG = "MLP"
RNN_TAG = "RNN"
RNN_ONE_HOT_TAG = "RNN_ONE_HOT"
RNN_ONE_HOT_INT_TAG = "RNN_ONE_HOT_INT"

RUN_TAGS = [MLP_TAG, RNN_TAG, RNN_ONE_HOT_TAG, RNN_ONE_HOT_INT_TAG]

PIPELINE_1, PIPELINE_2 = ['--pipeline-stage-1','--pipeline-stage-2']
SPECIAL_TAGS = [PIPELINE_1, PIPELINE_2]
SPECIAL_TAG_PRESENT = len([tag for tag in SPECIAL_TAGS if tag in sys.argv]) > 0


RUN_TYPE = RUN_TAGS.index(sys.argv[1]) if not SPECIAL_TAG_PRESENT and len(sys.argv) > 1 else RNN_TAG
RUN_TAG = sys.argv[1] if not SPECIAL_TAG_PRESENT and len(sys.argv) > 0 else RNN_TAG

SAVE_EVERY_N = 10

N_EPOCHS = 999999999
N_BATCHES = 10

DEFAULT_HIDDEN_LAYERS = 3
DEFAULT_NODES_PER_LAYER = 100

SPECIFIC_HYPER = None #[ (1, 120,30), (2,120,30), (3,120,30), (3,140,30), (3,180,30), ]

MAX_FILES_TESTING = None #100

MODEL_SAVE_FOLDER = "models/"
PREDICTIONS_SAVE_FOLDER = "predictions/"

RUNNING_ON_MAC = os.path.exists('./.on_mac')
RUNNING_ON_AWS = os.path.exists('./.on_aws')

RUN_NON_OCCLUDED = False

INPUT_FOLDER = 'committed_clouds/' if RUNNING_ON_MAC else 'clouds/'
if SPECIAL_TAG_PRESENT:
	INPUT_FOLDER = "totally_separate_clouds/"
if RUN_NON_OCCLUDED:
	INPUT_FOLDER = "non_occluded_"+INPUT_FOLDER

DATA_CACHE = '_classifier_input_' + RUN_TAG + '.pickle'
if RUN_NON_OCCLUDED:
	DATA_CACHE = "_non_occluded" + DATA_CACHE
COMPRESSED_DATA_CACHE = DATA_CACHE+".gz"

BREAK_IF_TEST_COST_IS_THRASHING = False


S3_FOLDER = "umbc.research/robot_learn_classifier/"
S3_DESTINATION = "s3://" + S3_FOLDER

def run_cmd(cmd):
	o,i = popen2.popen2(cmd)
	return o.read()


def get_label(lengths_line, run_type):
	lengths = [int(x) for x in lengths_line[len(LENGTHS_HEADER):].split("\t")]
	if run_type == RUN_RNN_ONE_HOT:
		label = [0] * LABEL_SIZE
		index = len(lengths) - MIN_LINKS
		label[index] = 1
		return label
	elif run_type == RUN_RNN_ONE_HOT_INT:
		label = [len(lengths)]
		return label
	
	while len(lengths) < MAX_LINKS:
		lengths.append(0.0)
	return lengths

def _flatten(lst):
	return [item for sublist in lst for item in sublist]

def _flatten_and_pad_points(skeleton_frame):
	flat = _flatten(skeleton_frame)
	while len(flat) < MAX_CENTROIDS * 3:
		flat.append(0.0)
	return flat

def prepare_skeleton(line):
	return _flatten_and_pad_points(get_skeleton_points(line))

def _shuffle_caches(cache_1, cache_2):
	combined = list(zip(cache_1, cache_2))
	random.shuffle(combined)
	return zip(*combined)

def _get_shuffled_batches(n_batches, cache_1, cache_2):
	batches = []
	batch_i = 0
	so_far = 0
	size = len(cache_1)
	batch_size = int(math.ceil(1.0 * size / n_batches));
	
	print "Creating",n_batches,"batches of",batch_size,"items, total of",size
	
	# do a shuffle on, randomness!
	shuffled_cache_1, shuffled_cache_2 = _shuffle_caches(cache_1, cache_2)
	
	while so_far < size:
		from_1 = shuffled_cache_1[so_far:so_far + batch_size]
		from_2 = shuffled_cache_2[so_far:so_far + batch_size]
		
		batches.append( (from_1, from_2) )
		
		batch_i += 1
		so_far += batch_size
	return batches

TEST_DATA = None
TRAINING_DATA = None
def data_label_batcher(data_cache, label_cache):
	global TEST_DATA, TRAINING_DATA
	
	if not TEST_DATA:
		
		batches = _get_shuffled_batches(N_BATCHES, data_cache, label_cache)
		
		if RUN_TYPE in [RUN_RNN, RUN_RNN_ONE_HOT]:
			# prune out bad data based on how many permutations we expect
			for i,batch in enumerate(batches):
				invalid_indexes = set([index for index,records in enumerate(batch[0]) if len(records) <> PERMUTATIONS])
				if len(invalid_indexes) > 0:
					new_data = [record for index,record in enumerate(batch[0]) if not index in invalid_indexes]
					new_labels = [record for index,record in enumerate(batch[1]) if not index in invalid_indexes]
			
					batches[i] = [new_data, new_labels]
		
		# separate out batches, only happens once
		TEST_DATA = batches[0]
		TRAINING_DATA = batches[1:]
	else:
		# shuffle the training data each round
		training_data = _flatten([first for first,_ in TRAINING_DATA])
		training_labels = _flatten([second for _,second in TRAINING_DATA])
		TRAINING_DATA = _get_shuffled_batches(N_BATCHES-1, training_data, training_labels)

	yield TEST_DATA
	for batch in TRAINING_DATA:
		yield batch

def load_data_cache(specific_link_count = None):
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

	if specific_link_count:
		filtered_data_cache = []
		filtered_label_cache = []
		for i,label in enumerate(label_cache):
			num_links = 0
			if RUN_TYPE == RUN_RNN_ONE_HOT:
				num_links = label.index(1) + MIN_LINKS
			elif RUN_TYPE in [RUN_RNN, RUN_MLP]:
				num_links = len(label) - label.count(0.0)

			if num_links <> specific_link_count:
				continue

			filtered_label_cache.append(label)
			filtered_data_cache.append(data_cache[i])
		print "filtered to", len(filtered_data_cache), "pairs with",specific_link_count,"links in", int(time.time() - start), "seconds"
		return (filtered_data_cache, filtered_label_cache)

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
	

def _prep_files(files, from_AWS):
	start = time.time()
	num_files = len(files)

	if not os.path.exists(INPUT_FOLDER):
		os.mkdir(INPUT_FOLDER)
	
	for i, file in enumerate(files):
		if file in [".DS_Store", ""]:
			continue
		
		downloaded = False
		local_file = INPUT_FOLDER + file
		if from_AWS and not os.path.exists(local_file):
			cmd = "aws s3 --region us-east-1 cp " + S3_DESTINATION + INPUT_FOLDER + file + " " + local_file
			print num_files,i,">",cmd
			run_cmd(cmd)
			downloaded = True
		
		if i % 1000 == 0:
			print round(time.time() - start,2), i, round(100.0 * i / len(files),2),"%",file
		yield file
		
		
		if from_AWS and downloaded:
			run_cmd("rm " + local_file)
		
		if MAX_FILES_TESTING and i >= MAX_FILES_TESTING:
			break

def get_data_labels(files, from_AWS = False):
	files = sorted(files)
	data = []
	labels = []
	other_labels = []
	all_pairs = []
	# files = random.shuffle(files)
	
	for file in _prep_files(files, from_AWS):
		all_pairs += [_ for _ in frame_reader(file)]
		#code.interact(local=dict(globals(), **locals())) 
	
	random.shuffle(all_pairs)
	
	data += [pair[0] for pair in all_pairs]
	labels += [pair[1] for pair in all_pairs]
	
	if SPECIAL_TAG_PRESENT:
		other_labels += [pair[2] for pair in all_pairs]
	
	return [data, labels, other_labels] if SPECIAL_TAG_PRESENT else [data, labels]

def frame_reader(file):
	with gzip.GzipFile(INPUT_FOLDER+file, 'r') as handle:
		skeleton_frame = None
		label = None
		other_label = None
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
				if RUN_TYPE in RNN_VARIATIONS or SPECIAL_TAG_PRESENT:
					skeleton_frames.append(skeleton_frame)
				continue
			
			if LENGTHS_HEADER in line and not SPECIAL_TAG_PRESENT:
				label = get_label(line, RUN_TYPE)
				continue
			elif LENGTHS_HEADER in line and SPECIAL_TAG_PRESENT:
				label = get_label(line, RUN_RNN_ONE_HOT)
				other_label = get_label(line, RUN_RNN)
				continue
			
		if RUN_TYPE == RUN_MLP and skeleton_frame and label:
			yield (skeleton_frame, label)
		elif (SPECIAL_TAG_PRESENT or RUN_TYPE in RNN_VARIATIONS) and label:
			if len(skeleton_frames) == PERMUTATIONS:
				yield (skeleton_frames, label, other_label) if SPECIAL_TAG_PRESENT else (skeleton_frames, label)
			else:
				print "invalid length seq:",file, len(skeleton_frames),"frames"

def mlp_model(x, n_input, class_length, hidden_layers, nodes_per_layer):
	if hidden_layers < 1:
		print "Hidden_layers must be at least 1"
		exit()
	
	# print n_input, class_length, hidden_layers, nodes_per_layer
	weights = {}
	biases = {}
	
	for i in xrange(hidden_layers):
		if i == 0:
			# first layer
			weights[i] = tf.Variable(tf.random_normal([ n_input, nodes_per_layer ]))			
			biases[i] = tf.Variable(tf.random_normal([nodes_per_layer])),
		elif i < hidden_layers:
			# internal layers
			weights[i] = tf.Variable(tf.random_normal([ nodes_per_layer, nodes_per_layer ]))
			biases[i] = tf.Variable(tf.random_normal([nodes_per_layer])),
		elif i == hidden_layers:
			# out layer
			weights[i] = tf.Variable(tf.random_normal([ nodes_per_layer, class_length ]))
			biases[i] = tf.Variable(tf.random_normal([class_length])),
	
	network = tf.nn.relu(tf.add(tf.matmul(x, weights[0]), biases[0]))
	for i in xrange(1, hidden_layers):
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
	
	
def rnn_one_hot_model(x, n_input, class_length, hidden_layers, nodes_per_layer):
	# RNN output node weights and biases
	weights = {
		'out': tf.Variable(tf.random_normal([nodes_per_layer, class_length]))
	}
	biases = {
		'out': tf.Variable(tf.random_normal([class_length]))
	}
	
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

	logits_series = tf.add(tf.matmul(outputs[-1], weights['out']), biases['out'])

	soft = tf.nn.softmax(logits_series)
	argmax_col = tf.argmax(soft, 1)
	pred = tf.one_hot(argmax_col, class_length, dtype = tf.int32)
	print "logits:", logits_series
	print "pred",pred
	print "argmax:",argmax_col
	print
	print
	
	return (logits_series, pred)

def get_rnn_one_hot_cost(logits_series, y):
	losses = tf.nn.softmax_cross_entropy_with_logits(logits = logits_series, labels = y)
	print "losses:",losses
	total_loss = tf.reduce_mean(losses)
	print "total loss:", total_loss
	return total_loss
	
	
	

# LINK_COUNT_WEIGHTING = 100
def get_mlp_cost(pred, y, class_length):
	print "pred:",pred
	print "Y:",y
	l2_distance = tf.sqrt( tf.reduce_sum(tf.square(tf.subtract(pred, y)), reduction_indices=1))
	penalty_sum = tf.reduce_sum(l2_distance)
	return penalty_sum


def get_rnn_cost(pred, y):
	print "pred:",pred
	print "Y:",y
	l2_distance = tf.sqrt( tf.reduce_sum(tf.square(tf.subtract(pred, y)), reduction_indices=1))
	penalty_sum = tf.reduce_sum(l2_distance)
	return penalty_sum


def savePredictions(prediction, ground_truth, batch_cost, predictions_folder, epoch):
	with open(PREDICTIONS_SAVE_FOLDER + predictions_folder + "/predicted.csv", "a") as predicted_file, \
		open(PREDICTIONS_SAVE_FOLDER + predictions_folder + "/loss.csv", "a") as loss_file, \
		open(PREDICTIONS_SAVE_FOLDER + predictions_folder + "/ground_truth.csv", "a") as actual_file:

		header = ["instance", "epoch",]
		if RUN_TYPE in [RUN_MLP, RUN_RNN]:
			for i in xrange(MAX_LINKS):
				header.append("link"+str(i))
		elif RUN_TYPE in [RUN_RNN_ONE_HOT]:
			for i in LINK_COUNTS:
				header.append(str(i)+"-link")

		if epoch is 1:
			actual_file.write(",".join(header)+"\n")
			predicted_file.write(",".join(header)+"\n")
			loss_file.write("epoch, total loss\n")
		for instance in range(0, len(prediction)):
			predicted_links = ','.join(map(str, prediction[instance]))
			predicted_file.write("arm_" + str(instance) + "," + str(epoch) + "," + predicted_links + "\n")
			
			links = ','.join(map(str, ground_truth[instance]))
			actual_file.write("arm_" + str(instance) + "," + str(epoch) + "," + links + "\n")
		loss_file.write(str(epoch) + "," + str(batch_cost) + "\n")


def run_batch(X, Y, sess, batch, cost, pred, predict, epoch, predictions_folder, optimizer = None):
	batch_cost = 0
	
	to_run = [cost]
	if optimizer:
		to_run.append(optimizer)
	
	can_predict = predict and predictions_folder

	prediction = []
	ground_truth = []

	if RUN_TYPE == RUN_MLP:
		epoch_x,epoch_y = batch
		results = sess.run(to_run, feed_dict = {X: epoch_x, Y: epoch_y})
		
		batch_cost = results[0]

		if can_predict:
			prediction.append(sess.run([pred], feed_dict = {X: epoch_x, Y: epoch_y})[0][0])
			ground_truth.append(epoch_y[0])

		if can_predict:
			savePredictions(prediction, ground_truth, batch_cost, predictions_folder, epoch)
		elif predict:
			print "WARNING: No predictions folder set."

	elif RUN_TYPE in RNN_VARIATIONS:
		data, labels = batch
		
		for i in xrange(len(data)):
			epoch_x = [data[i]]
			epoch_y = [labels[i]]
			training_epoch_y = epoch_y
			if RUN_TYPE == RUN_RNN_ONE_HOT:
				training_epoch_y = [labels[i]] * len(data[i])
			
			results = sess.run(to_run, feed_dict = {X: epoch_x, Y: training_epoch_y})
			batch_cost += results[0]

			if can_predict:
				prediction.append(sess.run([pred], feed_dict = {X: epoch_x, Y: training_epoch_y})[0][-1])
				ground_truth.append(epoch_y[0])

		if can_predict:
			savePredictions(prediction, ground_truth, batch_cost, predictions_folder, epoch)
		elif predict:
			print "WARNING: No predictions folder set."

	return batch_cost

def run_test(data_cache, label_cache, hidden_layers, nodes_per_layer, num_epochs, load_model_file = False, save_model = False, predictions_folder = False):
	tf.reset_default_graph()
	
	class_length = MAX_LINKS
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
	elif RUN_TYPE == RUN_RNN_ONE_HOT:
		class_length = LABEL_SIZE # because it's one-hot, of course
		X = tf.placeholder('float', [None, PERMUTATIONS, n_input])
		Y = tf.placeholder('float', [None, class_length])
		logits_series, pred = rnn_one_hot_model(X, n_input, class_length, hidden_layers, nodes_per_layer)
		cost = get_rnn_one_hot_cost(logits_series, Y)
	elif RUN_TYPE == RUN_RNN_ONE_HOT_INT:
		class_length = 1 # because it's a single integer vector
		X = tf.placeholder('float', [None, PERMUTATIONS, n_input])
		Y = tf.placeholder('float', [None, class_length])
		pred = rnn_model(X, n_input, class_length, hidden_layers, nodes_per_layer)
		cost = get_rnn_cost(pred, Y)
	
	optimizer = tf.train.AdamOptimizer(learning_rate=learning_rate).minimize(cost)

	# print "Trainable:", tf.trainable_variables()

	cost_stats = []

	overall_start = time.time()
	
	
	model_save_folder = MODEL_SAVE_FOLDER
	type_string = ""
	if RUN_TYPE == RUN_MLP:
		type_string = "MLP_"
		model_save_folder += "MLP_"
	elif RUN_TYPE == RUN_RNN:
		type_string = "RNN_"
		model_save_folder += "RNN_"
	elif RUN_TYPE == RUN_RNN_ONE_HOT:
		type_string = "RNN_ONE_HOT_"
		model_save_folder += "RNN_ONE_HOT_"
	elif RUN_TYPE == RUN_RNN_ONE_HOT_INT:
		type_string = "RNN_ONE_HOT_INT_"
		model_save_folder += "RNN_ONE_HOT_INT_"
	model_save_folder += str(int(overall_start)) + "_"
	model_save_folder += str(hidden_layers) + "_" + str(nodes_per_layer)
	

	saver = tf.train.Saver()
	
	stats_folder = "run_stats/run_" + type_string + str(int(time.time()))+"_"+str(hidden_layers)+"_"+str(nodes_per_layer)+"/"
	os.makedirs(stats_folder)
	saved = True
	prior_test_cost = None
	with open(stats_folder+'tf_results.csv', 'w') as handle:
		handle.write("\t".join(['Epoch', 'Epoch Train cost', 'Epoch Test cost', '1/Epoch Test cost'])+"\n")
		
		with tf.Session() as sess:
			# sess = tf_debug.LocalCLIDebugWrapperSession(sess)
			# you need to initialize all variables
			tf.global_variables_initializer().run()
		
			if load_model_file:
				load_current_state(saver, sess, load_model_file)
		
			# correct_prediction = tf.equal(tf.argmax(pred, 1), tf.argmax(Y, 1))
			# accuracy = tf.reduce_mean(tf.cast(correct_prediction, "float"))
			# accuracy = get_accuracy(pred, Y, class_length)
			for i in xrange(num_epochs):
				epoch_start = time.time()
				test_batch = None
				batcher = data_label_batcher(data_cache, label_cache)
			
				epoch_cost = 0
				num_batches = 0
			
				for j,batch in enumerate(batcher):
					if j == 0:
						test_batch = batch
						continue
					epoch_cost += run_batch(X, Y, sess, batch, cost, pred, False, i+1, predictions_folder, optimizer)
					num_batches += 1
					saved = False
				
				avg_train_cost = round(1.0 * epoch_cost / num_batches, 2) if num_batches > 0 else 0.0
			
				test_cost = run_batch(X, Y, sess, test_batch, cost, pred, True, i+1, predictions_folder)
				
				print ">", round(time.time() - overall_start,2), round(time.time() - epoch_start,2), i, j, "Avg epoch train cost:", avg_train_cost , "Test cost:", test_cost
				sys.stdout.flush()
				cost_line = "\t".join([str(x) for x in [i, avg_train_cost, test_cost, (1/test_cost if test_cost > 0 else 0)]])
				handle.write(cost_line + "\n")
				handle.flush()
				
				if not saved and save_model and i > 0 and SAVE_EVERY_N and i % SAVE_EVERY_N == 0:
					save_current_state(saver, sess, model_save_folder + "_" + str(i) + "/")
					saved = True
				
				if BREAK_IF_TEST_COST_IS_THRASHING and prior_test_cost and prior_test_cost < test_cost:
					break
				prior_test_cost = test_cost
				
			if not saved:
				save_current_state(saver, sess, model_save_folder + "_" + str(i) + "_final/")

	
	if RUNNING_ON_AWS:
		cmd = "aws s3 --region us-east-1 cp " + stats_folder + "* " + S3_DESTINATION + stats_folder
		print cmd
		run_cmd(cmd)
		run_cmd("rm -rf "+stats_folder)

def save_current_state(saver, sess, save_folder):
	if "/" <> save_folder[-1]:
		save_folder += "/"
	model_save_file = save_folder + "model.tf"
	print "Saving model as", model_save_file
	os.makedirs(save_folder)
	save_path = saver.save(sess, model_save_file)
	print "... Saved"
	if RUNNING_ON_AWS:
		cmd = "aws s3 --region us-east-1 cp --recursive " + save_folder + " " + S3_DESTINATION + save_folder
		print cmd
		run_cmd(cmd)

def load_current_state(saver, sess, load_folder):
	if "/" <> load_folder[-1]:
		load_folder += "/"
	model_save_file = load_folder + "model.tf"
	print "Loading model from", model_save_file
	saver.restore(sess, model_save_file)


def hyper_params():
	# num_layer_range, nodes_per_layer
	if RUN_TYPE == RUN_MLP:
		return [xrange(1,10), xrange(20, 200, 10)]
	elif RUN_TYPE == RUN_RNN:
		return [xrange(1,4), xrange(20, 200, 40)]
	elif RUN_TYPE == RUN_RNN_ONE_HOT:
		return [xrange(1,4), xrange(20, 200, 40)]
	elif RUN_TYPE == RUN_RNN_ONE_HOT_INT:
		return [xrange(1,4), xrange(20, 200, 40)]
	
	print "unknown hyper params"
	exit()

def run_hyper(data_cache, label_cache):
	
	if SPECIFIC_HYPER:
		print "Running specific hyper params"
		for layers, nodes_per_layer, num_epochs in SPECIFIC_HYPER:
			print "HYPER: ", layers, nodes_per_layer, num_epochs
			try:
				run_test(data_cache, label_cache, layers, nodes_per_layer, num_epochs, False, False)
			except Exception as error:
				print "Caught error, continuing"
				print error
				pass
		
		return
	
	layer_range, nodes_per_layer_range = hyper_params()
	
	for layers in layer_range:
		for nodes_per_layer in nodes_per_layer_range:
			print "HYPER: ", layers, nodes_per_layer
			try:
				run_test(data_cache, label_cache, layers, nodes_per_layer, N_EPOCHS, False, False)
			except Exception as error:
				print "Caught error, continuing"
				print error
				pass


def run_pipeline_stage_1():
	load_folder = "./models/RNN_ONE_HOT_1536967057_3_140_10"
	load_data = "_classifier_untouched_data.pickle"
	
	if not os.path.exists(load_data):
		files = os.listdir(INPUT_FOLDER)
		all_records = get_data_labels(files, False)
		cPickle.dump(all_records, open(load_data,'w'))
		exit()
	
	data,one_hot_labels,length_labels = cPickle.load(open(load_data,'r'))
	tf.reset_default_graph()
	
	n_input = MAX_CENTROIDS * 3
	
	hidden_layers = 3
	nodes_per_layer = 140
	
	class_length = LABEL_SIZE # because it's one-hot, of course
	X = tf.placeholder('float', [None, PERMUTATIONS, n_input])
	Y = tf.placeholder('float', [None, class_length])
	_, pred = rnn_one_hot_model(X, n_input, class_length, hidden_layers, nodes_per_layer)
	saver = tf.train.Saver()
	with tf.Session() as sess:
		tf.global_variables_initializer()
		
		load_current_state(saver, sess, load_folder)
		
		save_file = "pipeline1_results_"+str(int(time.time()))+".pickle"
		
		results = []
		num_links_offset = 2
		
		for i in xrange(len(data)):
			print i,len(data)
			sys.stdout.flush()
			predicted_label = list(sess.run([pred], feed_dict = {X: [data[i]]})[0][-1])
			one_hot_is_correct = predicted_label == one_hot_labels[i]
			
			predicted_links_int = predicted_label.index(1) + num_links_offset
			results.append([one_hot_labels[i], length_labels[i], data[i], predicted_links_int, one_hot_is_correct, predicted_label])
		
		cPickle.dump(results, open(save_file,'w'))
		print "Results saved to", save_file
		# code.interact(local=dict(globals(), **locals()))


def run_pipeline_stage_2(specific_model):
	models = {
		2 : "RNN_1534556276_3_140_99_final",
		3 : "RNN_1534586834_3_140_99_final",
		4 : "RNN_1534617307_3_140_99_final",
		5 : "RNN_1534648081_3_140_99_final",
		6 : "RNN_1534678961_3_140_99_final",
	}
	print "Running model with",specific_model,"links"
	
	load_folder = "./models/"+models[specific_model]
	
	phase_1_results = "pipeline1_results_1536973579.pickle" # to update
	all_results = cPickle.load(open(phase_1_results,'r'))
	
	tf.reset_default_graph()
	
	class_length = MAX_LINKS
	n_input = MAX_CENTROIDS * 3
	
	hidden_layers = 3
	nodes_per_layer = 140
	
	X = tf.placeholder('float', [None, PERMUTATIONS, n_input])
	Y = tf.placeholder('float', [None, class_length])
	pred = rnn_model(X, n_input, class_length, hidden_layers, nodes_per_layer)
	cost = get_rnn_cost(pred, Y)
	
	saver = tf.train.Saver()
	with tf.Session() as sess:
		tf.global_variables_initializer()
		
		load_current_state(saver, sess, load_folder)
		
		save_file = "pipeline2_results_"+str(int(time.time()))+"_"+str(specific_model)+".pickle"
		
		results = []
		
		for i,row in enumerate(all_results):
			
			true_one_hot_label, true_length_label, observation, predicted_links_int, one_hot_is_correct, predicted_one_hot_label = row
			
			if predicted_links_int <> specific_model:
				continue
			
			print i,len(all_results)
			sys.stdout.flush()
			
			prediction_length_vector = list(sess.run([pred], feed_dict = {X: [observation]})[0][-1])
			
			new_row = [true_one_hot_label, true_length_label, observation, predicted_links_int, one_hot_is_correct, predicted_one_hot_label, prediction_length_vector]
			
			#code.interact(local=dict(globals(), **locals()))
			results.append(new_row)
		
		cPickle.dump(results, open(save_file,'w'))
		print "Results saved to", save_file
		#code.interact(local=dict(globals(), **locals()))
	
	
	pass
		
	

if '__main__' == __name__:
	
	phase_1 = '--pipeline-stage-1'
	if phase_1 in sys.argv:
		run_pipeline_stage_1()
		exit()
	
	phase_2 = '--pipeline-stage-2'
	if phase_2 in sys.argv:
		specific_model = int(sys.argv[sys.argv.index(phase_2) + 1]) if len(sys.argv) > 2 and phase_2 in sys.argv else None
		if not specific_model:
			print "python",sys.argv[0],sys.argv[1],"<2,3,4,5,6>"
			exit()
		run_pipeline_stage_2(specific_model)
		exit()
	
	opt_load_saved_model = '-m'
	opt_save_prediction_to = '-p'
	opt_specific_link_count = '-l'
	
	example = "|".join(RUN_TAGS)
	
	if '-h' in sys.argv or '--help' in sys.argv or len(sys.argv) < 4:
		print "Usage: python", sys.argv[0], example,"num_hidden_layers num_nodes_per_layer"
		print "Usage: python", sys.argv[0], example,"num_hidden_layers num_nodes_per_layer <"+opt_load_saved_model +" saved_model_file> <"+opt_save_prediction_to+" prediction_folder> <"+opt_specific_link_count +" link-count>"
		print "Default: python", sys.argv[0], "RNN",DEFAULT_HIDDEN_LAYERS, DEFAULT_NODES_PER_LAYER
		exit()
	
	if not os.path.exists(COMPRESSED_DATA_CACHE):
		should_generate = True
		if RUNNING_ON_AWS:
			compressed_available = not "" == run_cmd("aws s3 --region us-east-1 ls " + S3_DESTINATION + COMPRESSED_DATA_CACHE)
			uncompressed_available = not "" == run_cmd("aws s3 --region us-east-1 ls " + S3_DESTINATION + DATA_CACHE)
			
			if compressed_available:
				print "Downloading compressed:", S3_DESTINATION + COMPRESSED_DATA_CACHE
				run_cmd("aws s3 --region us-east-1 cp " + S3_DESTINATION + COMPRESSED_DATA_CACHE + " .")
			if uncompressed_available:
				print "Downloading uncompressed:", S3_DESTINATION + DATA_CACHE
				run_cmd("aws s3 --region us-east-1 cp " + S3_DESTINATION + DATA_CACHE + " .")
			
			if compressed_available or uncompressed_available:
				print "Successfully downloaded from s3"
				should_generate = False
			else:
				print "Could not download pre-generated batch data from s3"
		
		if should_generate and RUNNING_ON_AWS:
			print "Generating data cache from AWS"
			input_files = run_cmd("aws s3 ls " + S3_DESTINATION + INPUT_FOLDER + " | ruby -e \"STDIN.readlines.each{|x| puts x.split.join(' ')}\" | cut -d' ' -f4").split("\n")
			gen_datacache(input_files, True)
		elif should_generate:
			print "Generating data cache from", INPUT_FOLDER
			input_files = os.listdir(INPUT_FOLDER)
			gen_datacache(input_files)
		exit()
	
	hidden_layers = DEFAULT_HIDDEN_LAYERS
	nodes_per_layer = DEFAULT_NODES_PER_LAYER

	specific_link_count = int(sys.argv[sys.argv.index(opt_specific_link_count)+1]) if opt_specific_link_count in sys.argv else None

	if len(sys.argv) >= 4:
		hidden_layers = int(sys.argv[2])
		nodes_per_layer = int(sys.argv[3])
	
	load_model_file = str(sys.argv[sys.argv.index(opt_load_saved_model)+1]) if opt_load_saved_model in sys.argv else None
	if load_model_file and not os.path.exists(load_model_file):
		print "Model load file does not exist:", load_model_file
		exit()

	save_prediction_folder = str(sys.argv[sys.argv.index(opt_save_prediction_to)+1]) if opt_save_prediction_to in sys.argv else None
	assert not save_prediction_folder or not os.path.exists(PREDICTIONS_SAVE_FOLDER + save_prediction_folder), "prediction folder already exists"
	if save_prediction_folder and not os.path.exists(PREDICTIONS_SAVE_FOLDER + save_prediction_folder):
		os.makedirs(PREDICTIONS_SAVE_FOLDER + save_prediction_folder)

	data_cache, label_cache = load_data_cache(specific_link_count)
	assert len(data_cache) > 0, "No data loaded"
	assert len(label_cache) > 0, "No labels loaded"
	
	# code.interact(local=dict(globals(), **locals()))
	if '--hyper' in sys.argv:
		run_hyper(data_cache, label_cache)
		sys.exit()
	
	print "Running",("MLP" if RUN_TYPE == RUN_MLP else "RNN"),"with", hidden_layers, "layers, each with", nodes_per_layer, "nodes"
	
	run_test(data_cache, label_cache, hidden_layers, nodes_per_layer, N_EPOCHS, load_model_file, True, save_prediction_folder)
	













