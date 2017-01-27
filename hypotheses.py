import sys, itertools, copy, cPickle, os, code, multiprocessing, random, math, time, hashlib,pprint
import numpy as np
from sets import Set

CACHE_FOLDER = 'caches/'

COMPUTE_PERMUTATIONS = False

REPLAY_FRAMES = '--replay-caches' in sys.argv

COMPUTE_FRAMES = not REPLAY_FRAMES

if len(sys.argv) < 4 or not '-s' in sys.argv or not '-p' in sys.argv:
    print "Usage: python",sys.argv[0]," <-t num-threads> -s skeleton_file.csv -p pointcloud.csv"
    print "example: python",sys.argv[0],"-s datasets/diverse_movement_skeleton.csv  -p datasets/diverse_movement_pcl.csv"
    print "example: python",sys.argv[0],"--replay-caches -s datasets/diverse_movement_skeleton.csv  -p datasets/diverse_movement_pcl.csv"
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

def get_fitness(edge_count, path_distance, total_pcl_error, num_points):
    edge_count_penalty = (-0.001 * num_points * edge_count)
    fit_to_data = (-1 * total_pcl_error)
    path_length_penalty = (-0.01 * path_distance)
    # print num_points, edge_count, path_distance, path_length_penalty, edge_count_penalty, fit_to_data
    # print "\t",edge_count_penalty + fit_to_data + path_length_penalty
    return edge_count_penalty + fit_to_data + path_length_penalty
    # return (-0.1 * joints) + (-0.1 * path_distance) + (-1 * total_pcl_error)

def get_all_pairs(lst):
    added = Set()
    for pair in itertools.permutations(lst, 2):
        ordered = tuple(sorted(pair))
        added.add(ordered)
    return added


CLOSEST_TO_VECTOR_MAP = {}
ERROR_MAP = {}
master_points_to_try = Set()
computed_skeleton_hash = None

def get_permutation_fitness(input_batch):
    global computed_skeleton_hash, CLOSEST_TO_VECTOR_MAP, ERROR_MAP, master_points_to_try
    
    output = []
    
    for input in input_batch:
        vertex_count, permutation, skeleton_points, pcl_points, new_skeleton_hash = input
        
        if not new_skeleton_hash == computed_skeleton_hash:
            computed_skeleton_hash = new_skeleton_hash
            master_points_to_try = Set()
            
            all_vectors = get_all_pairs(skeleton_points)
            for point in pcl_points:
                master_points_to_try.add(point)
                
                # print ">>",point
                near_error,near_vector = get_distance_to_nearest_vector(point, all_vectors)
                
                if not near_vector in CLOSEST_TO_VECTOR_MAP:
                    CLOSEST_TO_VECTOR_MAP[near_vector] = Set()
                    ERROR_MAP[near_vector] = 0.0
                    
                    #print "<<",near_vector
                
                CLOSEST_TO_VECTOR_MAP[near_vector].add(point)
                ERROR_MAP[near_vector] += near_error
            # print "COMPUTED VECTOR AND ERROR MAPS",len(ERROR_MAP),"of",len(all_vectors)
        
        path = tuple([skeleton_points[i] for i in permutation])
    
        distance = calculate_distances(path)
    
        # code.interact(local=dict(globals(), **locals()))
    
        vectors_endpoints = sorted([ (path[i+1],path[i]) for i in range(len(path) - 1) ])
        #print "vector endpoints:",len(vectors_endpoints)
        total_error = 0.0

        pcl_hash = hashlib.md5(str(pcl_points)).hexdigest()
        
        points_to_try = Set(list(master_points_to_try))
        
        for vector in vectors_endpoints:
            p0,p1 = sorted(vector)
            hash = (p0,p1)
            #print ">>",hash
            if hash in ERROR_MAP:
                total_error += ERROR_MAP[hash] # get the error for all the closest points we already computed the sum for
                for point in CLOSEST_TO_VECTOR_MAP[hash]:
                    points_to_try.discard(point)
        
        #print "leftover:",len(points_to_try),"of",len(master_points_to_try)
        
        for point in points_to_try:
            min_distance = None
            for vector in vectors_endpoints:
                hash = tuple(['o'] + (sorted( vector )) + [point])
                this_distance = DISTANCES_CACHE[hash]
                if not min_distance or this_distance < min_distance:
                    min_distance = this_distance
            total_error += min_distance
                
        # code.interact(local=dict(globals(), **locals()))
        
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
        
        cPickle.dump(without_reverse_paths, open(cache_file,'wb'))
        print "\tsaved",cache_file
    
        permutations = list(without_reverse_paths)
    elif not COMPUTE_PERMUTATIONS:
        permutations = cPickle.load(open(cache_file, 'rb'))

    if COMPUTE_PERMUTATIONS:
        return None
    
    start = time.time()
    combined = []
    
    new_skeleton_hash = hashlib.md5( str(tuple(sorted(skeleton_points)))).hexdigest()
    
    # don't try using permutations generated with more vertices than you have, avoid indexing outside list
    inputs = [(vertex_count, permutation, skeleton_points, pcl_points, new_skeleton_hash) for permutation in permutations if max(permutation) < to_permute]
    
    computed = None
    
    chunk_size = max(1, len(inputs) / NUM_THREADS)
    
    if 1 == NUM_THREADS:
        computed = [get_permutation_fitness(_) for _ in chunks(inputs, chunk_size)]
    else:
        computed = pool.map(get_permutation_fitness, chunks(inputs, chunk_size))
    
    for path_batch in computed:
        for path in path_batch:
            combined.append(path)
    
    best_first = sorted(combined, key=lambda x:x[1], reverse=1)
    
    taken = time.time() - start
    print "\tTaken:",round(taken, 2),"seconds, Permutations/sec:",round(len(permutations) / taken, 2),"for",len(inputs),"permutations"
    
    return best_first

def do_analysis():

    input_skeleton = sys.argv[sys.argv.index('-s')+1]
    input_pointcloud = sys.argv[sys.argv.index('-p')+1]
    precision_digits = 3
    
    pool = multiprocessing.Pool(NUM_THREADS)
    
    # somewhat arbitrary value for bounding concerns
    max_edges = 6 #if NUM_THREADS > 8 else 4
    
    start_id = str(int(time.time()))
    
    best_case_output_file = "best_values_"+start_id+".csv"
    
    columns = ["Frame"] + [str(x) for x in range(1, max_edges+1)]
    open(best_case_output_file,'w').write("\t".join(columns)+"\n")
    
    best_path_output = "best_path_"+start_id+".csv"
    open(best_path_output, 'w').write('Frame\tPath\n')
    
    max_points_to_use = 500
    
    #random.seed(1)
    
    so_far = 0
    for skeleton_points, pcl_points in get_frames(input_skeleton, input_pointcloud):
        frame_start = time.time()
        print SENTINEL,"frame:",so_far
        so_far += 1

        # code.interact(local=dict(globals(), **locals()))
        
        points_to_use = min(max_points_to_use, len(pcl_points))
        sampled_pcl_points = random.sample(pcl_points, points_to_use)
        print "Using",len(sampled_pcl_points),"of",len(pcl_points),"Pointcloud points"
        
        skeleton_points = [round_to_precision(point, precision_digits) for point in skeleton_points]
        sampled_pcl_points = [round_to_precision(point, precision_digits) for point in sampled_pcl_points]
        
        bests = []
        
        for edge_count in range(1,max_edges+1):
            # go in reverse order, most -> least
            # edge_count = (max_edges+1) - edge_count

            if edge_count >= len(skeleton_points):
                continue
            
            input_tuple = (skeleton_points, pcl_points, edge_count)
            input_hash = hashlib.md5(str(input_tuple)).hexdigest()
            cache_file = CACHE_FOLDER+"_input_"+input_hash+".pickle"
            permuted_paths = None
            vertex_count = edge_count+1
            
            print "vertex_count:", vertex_count, "points:",len(skeleton_points)
            
            cache_file_exists = os.path.exists(cache_file)
            
            if not REPLAY_FRAMES and (COMPUTE_PERMUTATIONS or not cache_file_exists):
                permuted_paths = get_paths(pool, skeleton_points, sampled_pcl_points, vertex_count)
                if not COMPUTE_PERMUTATIONS:
                    cPickle.dump(permuted_paths, open(cache_file,'wb'))
                    print "\tSaved paths to",cache_file
            elif not COMPUTE_FRAMES and not COMPUTE_PERMUTATIONS and cache_file_exists:
                try:
                    print "\tLOADING CACHE",cache_file
                    permuted_paths = cPickle.load(open(cache_file,'rb'))
                    print "\tdone"
                except ValueError:
                    print "Caught cPickle error, bugged file:",cache_file
                    permuted_paths = None
            
            if not permuted_paths:
                exit()
            
            best_score = permuted_paths[0][1]
            print "\tbest:",best_score
            bests.append(best_score)
            path_strings = [",".join([str(x) for x in path]) for path in permuted_paths[0][0]]
            # code.interact(local=dict(globals(), **locals()))
            
            open(best_path_output,'a').write("\t".join([str(so_far), str(edge_count)]+path_strings)+"\n")
                
            #exit()
            
            if REPLAY_FRAMES and not permuted_paths:
                exit()
            
            if COMPUTE_FRAMES or COMPUTE_PERMUTATIONS:
                continue
            
            #code.interact(local=dict(globals(), **locals()))

        open(best_case_output_file, 'a').write("\t".join([str(so_far)] + [str(x) for x in bests])+"\n")
        print ">> frame took",round(time.time() - frame_start, 2),"seconds"
    

if '__main__' == __name__:
    
    if not os.path.exists(CACHE_FOLDER):
        os.mkdir(CACHE_FOLDER)
    
    do_analysis()
    
    
    