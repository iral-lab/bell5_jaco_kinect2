import sys, itertools, copy, cPickle, os, code, multiprocessing, random, math, time, hashlib
import numpy as np
from sets import Set

CACHE_FOLDER = 'caches/'

COMPUTE_INITIAL_FRAME_PARSE = True

if len(sys.argv) < 4 or not '-s' in sys.argv or not '-p' in sys.argv:
    print "Usage: python",sys.argv[0]," <-t num-threads> -s skeleton_file.csv -p pointcloud.csv"
    print "example: python hypotheses.py -s datasets/diverse_movement_skeleton.csv  -p datasets/diverse_movement_pcl.csv"
    exit()

NUM_THREADS = int(sys.argv[sys.argv.index('-t')+1]) if '-t' in sys.argv else 8
print "Running with",NUM_THREADS,"threads"

SENTINEL = "==============="
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
    hash = ('o',p0,p1,x)
    if not hash in DISTANCES_CACHE:
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
        
        DISTANCES_CACHE[hash] = DISTANCES_CACHE[hash_len_cross] / DISTANCES_CACHE[hash_len_p1_p0]
    return DISTANCES_CACHE[hash]

def get_distance_to_nearest_vector(point, vector_endpoints):
    distances = [distance_to_vector(p0,p1, point) for p0,p1 in vector_endpoints]
    return min(distances)
    
def get_fitness(joints, path_distance, total_pcl_error, num_points):
    return (-0.05 * num_points * joints) + (-0.1 * path_distance) + (-1 * total_pcl_error)

def get_permutation_fitness(input_batch):
    output = []
    for input in input_batch:
        vertex_count, permutation, skeleton_points, pcl_points = input
    
        path = tuple([skeleton_points[i] for i in permutation])
    
        distance = calculate_distances(path)
    
        # code.interact(local=dict(globals(), **locals()))
    
        vectors_endpoints = [ (path[i+1],path[i]) for i in range(len(path) - 1) ]
    
        errors = [get_distance_to_nearest_vector(point, vectors_endpoints) for point in pcl_points]
        total_error = sum(errors)
    
        fitness = get_fitness(vertex_count-1, distance, total_error, len(pcl_points))
    
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

PERMUTATIONS_FILE = CACHE_FOLDER+"_permutations"
def get_paths(pool, skeleton_points, pcl_points, vertex_count):
    
    to_permute = len(skeleton_points)
    cache_file = PERMUTATIONS_FILE+"_"+str(vertex_count)+"_"+str(to_permute)+".pickle"
    
    if not os.path.exists(cache_file):
        indices = range(to_permute)
        stack = []
        for i,x in enumerate(indices):
            rest = indices[:i]+indices[i+1:]
            stack.append( ([x], rest, vertex_count, to_permute) )
        
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
        
        cPickle.dump(without_reverse_paths, open(cache_file,'wb'))
        print "\tsaved",cache_file
    else:
        permutations = cPickle.load(open(cache_file, 'rb'))
    
    if COMPUTE_INITIAL_FRAME_PARSE:
        return None
    
    start = time.time()
    combined = []
    
    # don't try using permutations generated with more vertices than you have, avoid indexing outside list
    inputs = [(vertex_count, permutation,skeleton_points, pcl_points)  for permutation in permutations if max(permutation) < to_permute]
    
    computed = pool.map(get_permutation_fitness, chunks(inputs, NUM_THREADS))
    
    for path_batch in computed:
        for path in path_batch:
            combined.append(path)
    
    best_first = sorted(combined, key=lambda x:x[1], reverse=1)
    
    taken = time.time() - start
    print "\tTaken:",round(taken, 2),"Permutations/sec:",round(len(permutations) / taken, 2)
    
    return best_first
    

if '__main__' == __name__:
    input_skeleton = sys.argv[sys.argv.index('-s')+1]
    input_pointcloud = sys.argv[sys.argv.index('-p')+1]
    
    if not os.path.exists(CACHE_FOLDER):
        os.mkdir(CACHE_FOLDER)

    pool = multiprocessing.Pool(NUM_THREADS)
    
    # somewhat arbitrary value for bounding concerns
    max_edges = 7

    pcl_validation_point_percentage = 0.2
    
    #random.seed(1)
    
    so_far = 0
    for skeleton_points, pcl_points in get_frames(input_skeleton, input_pointcloud):
        print SENTINEL,"frame:",so_far
        so_far += 1
        sampled_pcl_points = random.sample(pcl_points, int(pcl_validation_point_percentage * len(pcl_points)))
        print "Using",len(sampled_pcl_points),"of",len(pcl_points),"Pointcloud points"
        for edge_count in range(1,max_edges+1):
            if edge_count >= len(skeleton_points):
                continue
            
            input_tuple = (skeleton_points, pcl_points, edge_count)
            input_hash = hashlib.md5(str(input_tuple)).hexdigest()
            cache_file = CACHE_FOLDER+"_input_"+input_hash+".pickle"
            permuted_paths = None
            vertex_count = edge_count+1
            
            print "vertex_count:", vertex_count, "points:",len(skeleton_points)
            
            if os.path.exists(cache_file):
                permuted_paths = cPickle.load(open(cache_file,'rb'))
                print "\tLOADED CACHE",cache_file
            else:
                permuted_paths = get_paths(pool, skeleton_points, sampled_pcl_points, vertex_count)
                if not COMPUTE_INITIAL_FRAME_PARSE:
                    cPickle.dump(permuted_paths, open(cache_file,'wb'))
                    print "\tSaved paths to",cache_file
            if not COMPUTE_INITIAL_FRAME_PARSE:
                print "\tbest:",permuted_paths[0][1]
            #code.interact(local=dict(globals(), **locals()))
            
            # if edge_count == 4:
            #     exit()
            #break
        #break
        # if so_far == 2:
        #     exit()
