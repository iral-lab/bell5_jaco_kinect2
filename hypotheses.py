import sys, itertools, copy, cPickle, os, code, multiprocessing, random, math, time, hashlib, pprint, math, marshal, json
import numpy as np
from sets import Set

COMPRESSION = cPickle #json #marshal #cPickle
COMPRESSION_EXTENSION = '.pickle'

CACHE_FOLDER = 'caches/'

# somewhat arbitrary value for bounding concerns
MAX_EDGES = 6 #if NUM_THREADS > 8 else 4

REPLAY_FRAMES = '--replay-caches' in sys.argv

COMPUTE_FRAMES = not REPLAY_FRAMES

NUM_THREADS = int(sys.argv[sys.argv.index('-t')+1]) if '-t' in sys.argv else 8


SENTINEL = "===="
def csv_reader(input_file):
	with open(input_file, 'r') as handle:
		next_line = handle.readline()
		batch = []
		while next_line:
			line = next_line.strip()
			next_line = handle.readline()

			if SENTINEL in line:
				yield batch
				batch = []
			else:
				batch.append( tuple([float(x) for x in line.split(",")]) )
		if len(batch) > 0:
			yield batch

def round_to_precision(point, precision):
	return tuple([round(value, precision) for value in point])

def get_frames(skeleton_file, pcl_file):
	skeleton_in = csv_reader(skeleton_file)
	pcl_in = csv_reader(pcl_file)
	
	skeleton = skeleton_in.next()
	pcl = pcl_in.next()
	
	while skeleton and pcl:
		yield (skeleton, pcl)	
		skeleton = skeleton_in.next()
		pcl = pcl_in.next()
	
	
def euclid_distance(p1,p2):
	return np.linalg.norm(np.array(p1) - np.array(p2))


def calculate_distances(points):
	sum_distance = 0.0
	for i in range(len(points) - 1):
		start,stop = points[i:i+2]
		sum_distance += euclid_distance(start,stop)
	return sum_distance

def vector_between(p0, p1):
	return tuple([p0[i] - p1[i] for i in range(len(p1))])

def length_3d(vector_3d):
	x,y,z = vector_3d
	return math.sqrt( x*x + y*y + z*z )

DISTANCES_CACHE = {}
def distance_to_vector(p0, p1, x):
	# from http://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
	
	hash = tuple(['o'] + sorted( [p0, p1] ) + [x])
	
	if hash in DISTANCES_CACHE:
		return DISTANCES_CACHE[hash]
		
	hash_x_minus_p0 = ('m',x,p0)
	hash_x_minus_p1 = ('m',x,p1)
	hash_len_p1_p0 = ('d',p1,p0)
	hash_cross_xp0_xp1 = ('c',x,p1,p0)
	hash_len_cross = ('l',x,p0,p1)
	
	if not hash_x_minus_p0 in DISTANCES_CACHE:
		DISTANCES_CACHE[hash_x_minus_p0] = vector_between(x, p0)
	if not hash_x_minus_p1 in DISTANCES_CACHE:
		DISTANCES_CACHE[hash_x_minus_p1] = vector_between(x, p1)
	if not hash_len_p1_p0 in DISTANCES_CACHE:
		DISTANCES_CACHE[hash_len_p1_p0] = euclid_distance(p1,p0)
	
	if not hash_cross_xp0_xp1 in DISTANCES_CACHE:
		DISTANCES_CACHE[hash_cross_xp0_xp1] = np.cross(DISTANCES_CACHE[hash_x_minus_p0], DISTANCES_CACHE[hash_x_minus_p1])
	
	# from https://docs.scipy.org/doc/numpy-1.10.0/reference/generated/numpy.cross.html
	if not hash_len_cross in DISTANCES_CACHE:
		DISTANCES_CACHE[hash_len_cross] = length_3d(DISTANCES_CACHE[hash_cross_xp0_xp1])
	
	distance = DISTANCES_CACHE[hash_len_cross] / DISTANCES_CACHE[hash_len_p1_p0]
	DISTANCES_CACHE[hash] = distance
	return distance

def get_distance_to_nearest_vector_parallel(input):
	return get_distance_to_nearest_vector(input[0], input[1])

def get_distance_to_nearest_vector(point, vector_endpoints):
	
	min_distance = None
	closest_to = None
	for p0,p1 in vector_endpoints:
		p0,p1 = sorted([p0,p1])
		hash = tuple(['o', p0, p1, point])
	
		distance = None
		if hash in DISTANCES_CACHE:
			distance = DISTANCES_CACHE[hash]
		else:
			distance = distance_to_vector(p0, p1, point)
		if not min_distance or distance < min_distance:
			min_distance = distance
			closest_to = (p0, p1)
	return (min_distance, closest_to)

def get_fitness(edge_count, path_distance, total_pcl_error):
	
	fit_to_data = -10000.0 * total_pcl_error
	
	lambda_scalar = 2.0 # ridiculously high, but should show some effect as they get higher
	edge_count_penalty = math.exp(lambda_scalar * edge_count)
	
	total_penalty = fit_to_data - edge_count_penalty
	
	# print "path penalty:",fit_to_data, total_pcl_error, edge_count, edge_count_penalty, total_penalty
	
	return total_penalty

def get_all_pairs(lst):
	added = Set()
	for pair in itertools.permutations(lst, 2):
		ordered = tuple(sorted(pair))
		added.add(ordered)
	return added

def get_permutation_fitness(input_batch):
	output = []
	permutation_batch, skeleton_points, lookup_cache_file = input_batch
	lookup = load_cache_file(lookup_cache_file)

	for permutation in permutation_batch:
		
		path = tuple([skeleton_points[i] for i in permutation])
		
		distance = calculate_distances(path)

		vectors_endpoints = tuple(sorted([ tuple(sorted((path[i+1],path[i]))) for i in range(len(path) - 1) ]))
		
		#print "vector endpoints:",len(vectors_endpoints)
		total_error = lookup[vectors_endpoints]
		
		fitness = get_fitness(len(path), distance, total_error)
	
		# add back in the opposite path
		opposite_path = list(path)
		opposite_path.reverse()
		output.append( (path, fitness) )
		output.append( (tuple(opposite_path), fitness) )
	return output

def chunks(l, n):
	"""Yield successive n-sized chunks from l."""
	for i in range(0, len(l), n):
		yield l[i:i + n]

def build_perm(input):
	start, rest, vertex_count, to_permute = input
	stack = [ (start,rest) ]
	permutations = []

	indices = range(to_permute)
	indices_set = Set(indices)
	
	while len(stack) > 0:
		path, rest = stack.pop()
		
		if len(path) == vertex_count:
			permutations.append(tuple(path))
			continue
	
		remaining = indices_set - Set(path)
		for next_index in remaining:
			new_path = copy.deepcopy(path)
			new_path.append(next_index)
			stack.append( (new_path, Set(remaining) - Set([next_index]) ) )
	return permutations

def flatten(l):
	return [x for subl in l for x in subl]


def iterate_distance_for_perm(input):
	perm, sorted_skeleton_points, pcl_points = input
	
	edges = tuple([sorted_skeleton_points[i] for i in perm])
	unsorted_endpoints = [ tuple(sorted((edges[i], edges[i+1]))) for i in range(len(edges) - 1)]
	vector_endpoints = tuple(sorted(unsorted_endpoints))
	
	# code.interact(local=dict(globals(), **locals()))
	error = 0.0
	for point in pcl_points:
		min_distance,closest = get_distance_to_nearest_vector(point, vector_endpoints)
		error += min_distance
	# print error
	# code.interact(local=dict(globals(), **locals()))
	
	return (vector_endpoints, error)

LOOKUP_FILE = CACHE_FOLDER+"_lookup"
LOOKUPS = {} # stores file name, since other threads have to load it
def build_distance_lookup_table(pool, skeleton_points, pcl_points):
	lookup_hash = hashlib.md5( str(tuple(sorted(skeleton_points))) + str(tuple(sorted(pcl_points)))).hexdigest()
	cache_file = LOOKUP_FILE+"_"+str(MAX_EDGES)+"_"+str(lookup_hash)+COMPRESSION_EXTENSION
	
	lookup = None
	if not os.path.exists(cache_file):
		lookup = {}
		all_edges = sorted(get_all_pairs(skeleton_points))
		sorted_skeleton_points = sorted(skeleton_points)
		to_permute = len(skeleton_points)
		start = time.time()
		for vertex_count in range(2, min(MAX_EDGES+1, to_permute)+1):
			this_round = time.time()
			print vertex_count, to_permute
			permutations = load_or_build_perms(pool, vertex_count, to_permute)
		
			print ">",len(permutations)
		
			inputs = [(perm, sorted_skeleton_points, pcl_points) for perm in permutations]
			results = pool.map(iterate_distance_for_perm, inputs)
			#results = [iterate_distance_for_perm(input) for input in inputs]
		
			#code.interact(local=dict(globals(), **locals()))
			temp_lookup = dict(results)
			lookup.update(temp_lookup)
			print round(time.time() - start,2),round(time.time() - this_round, 2), ">>",len(temp_lookup),len(lookup)
		
		open(cache_file,'wb').write(COMPRESSION.dumps(lookup))
		print "\tsaved",cache_file
		LOOKUPS[lookup_hash] = cache_file
	elif not lookup_hash in LOOKUPS:
		# print "\tloading",cache_file
		LOOKUPS[lookup_hash] = cache_file # load_cache_file(cache_file)
	else:
		print "saved lookup load"
	return LOOKUPS[lookup_hash]
	
FILE_CACHE = {}
def load_cache_file(path):
	if not path in FILE_CACHE:
		FILE_CACHE[path] = COMPRESSION.loads(open(path, 'rb').read())
	return FILE_CACHE[path]
	
	
def load_or_build_perms(pool, vertex_count, to_permute):
	cache_file = PERMUTATIONS_FILE+"_"+str(vertex_count)+"_"+str(to_permute)+COMPRESSION_EXTENSION
	permutations = None
	if not os.path.exists(cache_file):
		indices = range(to_permute)
		stack = []
		temp_stack = []
		for i,x in enumerate(indices):
			rest = indices[:i]+indices[i+1:]
			temp_stack.append( ([x], rest, vertex_count, to_permute) )
		
		# do one pass of the alg to seed parallel threads
		for input in temp_stack:
			path,rest,vertex_count,to_permute = input
			for i,x in enumerate(rest):
				new_path = copy.deepcopy(path)
				new_path.append(x)
				new_rest = rest[:i]+rest[i+1:]
				stack.append( (new_path, new_rest, vertex_count, to_permute) )
		
		
		permutations = pool.map(build_perm, stack)
		# permutations = [build_perm(x) for x in stack]
		flattened = flatten(permutations)

		without_reverse_paths = Set()
		for perm in flattened:
			perm = tuple(perm)
			opposite = list(perm)
			opposite.reverse()
			opposite = tuple(opposite)
			if opposite in without_reverse_paths:
				continue
			without_reverse_paths.add(perm)
		
		permutations = sorted(list(without_reverse_paths))
		open(cache_file,'wb').write(COMPRESSION.dumps(permutations))
		print "\tsaved",cache_file
	
		
	else:
		permutations = COMPRESSION.loads(open(cache_file, 'rb').read())
	return sorted(permutations)

PERMUTATIONS_FILE = CACHE_FOLDER+"_permutations"
def get_paths(pool, skeleton_points, pcl_points, vertex_count):
	
	to_permute = len(skeleton_points)
	permutations = load_or_build_perms(pool, vertex_count, to_permute)
	
	start = time.time()
	
	lookup_cache_file = build_distance_lookup_table(pool, skeleton_points, pcl_points)
	
	combined = []
	
	new_skeleton_hash = hashlib.md5( str(tuple(sorted(skeleton_points)))).hexdigest()
	
	chunk_size = max(1, len(permutations) / NUM_THREADS)
	
	# don't try using permutations generated with more vertices than you have, avoid indexing outside list
	#inputs = [(permutation, skeleton_points, lookup) for permutation in permutations if max(permutation) < to_permute]
	permutations_batches = chunks(permutations, chunk_size)
	inputs = [(permutation_batch, skeleton_points, lookup_cache_file) for permutation_batch in permutations_batches]
	
	print "input_stats:",NUM_THREADS, len(permutations), chunk_size, len(inputs)
	
	computed = None
	
	if 1 == NUM_THREADS:
		computed = [get_permutation_fitness(_) for _ in inputs]
	else:
		computed = pool.map(get_permutation_fitness, inputs)
	
	try:
		print "Lookup file deleted",lookup_cache_file
		os.remove(lookup_cache_file)
	except:
		pass
	
	for path_batch in computed:
		for path in path_batch:
			combined.append(path)
	
	best_first = sorted(combined, key=lambda x:x[1], reverse=1)
	
	taken = time.time() - start
	print "\tTaken:",round(taken, 2),"seconds, Permutations/sec:",round(len(permutations) / taken, 2),"for",len(permutations),"permutations"
	
	return best_first

def clear_caches(n):
	FILE_CACHE = {}
	LOOKUPS = {}
	DISTANCES_CACHE = {}

def do_analysis():

	input_skeleton = sys.argv[sys.argv.index('-s')+1]
	input_pointcloud = sys.argv[sys.argv.index('-p')+1]
	precision_digits = 3
	
	pool = multiprocessing.Pool(NUM_THREADS)
		
	start_id = str(int(time.time()))
	
	best_case_output_file = "best_values_"+start_id+".csv"
	
	columns = ["Frame"] + [str(x) for x in range(1, MAX_EDGES+1)]
	open(best_case_output_file,'w').write("\t".join(columns)+"\n")
	
	best_path_output = "best_path_"+start_id+".csv"
	open(best_path_output, 'w').write('Frame\tPath\n')
	
	max_points_to_use = 500
	
	random.seed(1)
	
	so_far = 0
	for skeleton_points, pcl_points in get_frames(input_skeleton, input_pointcloud):
		frame_start = time.time()
		print SENTINEL,"frame:",so_far
		so_far += 1

		# code.interact(local=dict(globals(), **locals()))
		
		pool.map(clear_caches, range(NUM_THREADS))
		
		points_to_use = min(max_points_to_use, len(pcl_points))
		sampled_pcl_points = random.sample(pcl_points, points_to_use)
		print "Using",len(sampled_pcl_points),"of",len(pcl_points),"Pointcloud points"
		
		skeleton_points = [round_to_precision(point, precision_digits) for point in skeleton_points]
		sampled_pcl_points = [round_to_precision(point, precision_digits) for point in sampled_pcl_points]
		
		bests = []
		
		for edge_count in range(1,MAX_EDGES+1):
			# go in reverse order, most -> least
			# edge_count = (MAX_EDGES+1) - edge_count

			if edge_count >= len(skeleton_points):
				continue
			
			input_tuple = (skeleton_points, pcl_points, edge_count)
			input_hash = hashlib.md5(str(input_tuple)).hexdigest()
			cache_file = CACHE_FOLDER+"_frame_"+input_hash+COMPRESSION_EXTENSION
			permuted_paths = None
			vertex_count = edge_count+1
			
			print "vertex_count:", vertex_count, "points:",len(skeleton_points)
			
			cache_file_exists = os.path.exists(cache_file)
			
			if cache_file_exists or (REPLAY_FRAMES and cache_file_exists):
				try:
					print "\tLOADING CACHE",cache_file
					permuted_paths = COMPRESSION.loads(open(cache_file,'rb').read())
					print "\tdone"
				except ValueError:
					print "Caught COMPRESSION error, bugged file:",cache_file
					permuted_paths = None
			
			else:
				permuted_paths = get_paths(pool, skeleton_points, sampled_pcl_points, vertex_count)
				open(cache_file,'wb').write(COMPRESSION.dumps(permuted_paths))
				print "\tSaved paths to",cache_file	
			
			if not permuted_paths:
				exit()
			
			best_score = permuted_paths[0][1]
			print "\tbest:",best_score
			bests.append(best_score)
			path_strings = [",".join([str(x) for x in path]) for path in permuted_paths[0][0]]
			# code.interact(local=dict(globals(), **locals()))
			
			open(best_path_output,'a').write("\t".join([str(so_far), str(edge_count)]+path_strings)+"\n")
			
			if REPLAY_FRAMES and not permuted_paths:
				exit()
			
			if COMPUTE_FRAMES:
				continue
			
			#code.interact(local=dict(globals(), **locals()))

		open(best_case_output_file, 'a').write("\t".join([str(so_far)] + [str(x) for x in bests])+"\n")
		print ">> frame took",round(time.time() - frame_start, 2),"seconds"
	

if '__main__' == __name__:

	if len(sys.argv) < 4 or not '-s' in sys.argv or not '-p' in sys.argv:
		print "Usage: python",sys.argv[0]," <-t num-threads> -s skeleton_file.csv -p pointcloud.csv"
		print "example: python",sys.argv[0],"-s datasets/diverse_movement_skeleton.csv  -p datasets/diverse_movement_pcl.csv"
		print "example: python",sys.argv[0],"--replay-caches -s datasets/diverse_movement_skeleton.csv  -p datasets/diverse_movement_pcl.csv"
		exit()
	
	print "Running with",NUM_THREADS,"threads"
	
	if not os.path.exists(CACHE_FOLDER):
		os.mkdir(CACHE_FOLDER)
	
	do_analysis()
	
	
	