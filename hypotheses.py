import sys, itertools, copy, marshal, os, code, multiprocessing, random, math
import numpy as np
from sets import Set

NUM_THREADS = 8

if len(sys.argv) < 3:
	print "Usage: python",sys.argv[0],"skeleton_file.csv pointcloud.csv"
	exit()

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

def distance_to_vector(p0, p1, x):
    # from http://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
    x_minus_p0 = vector_between(x, p0)
    x_minus_p1 = vector_between(x, p1)
    len_p1_p0 = euclid_distance(p1,p0)
    cross_xp0_xp1 = np.cross(x_minus_p0, x_minus_p1)
    
    # from https://docs.scipy.org/doc/numpy-1.10.0/reference/generated/numpy.cross.html
    len_cross = length_3d(cross_xp0_xp1)
    
    return len_cross / len_p1_p0

def get_distance_to_nearest_vector(point, vector_endpoints):
    distances = [distance_to_vector(p0,p1, point) for p0,p1 in vector_endpoints]
    return min(distances)
    
def get_fitness(joints, path_distance, total_pcl_error):
    return (-0.1 * joints) + (-0.1 * path_distance) + (-1 * total_pcl_error)

def get_permutation_fitness(input):
    vertex_count, permutation, skeleton_points, pcl_points = input
    
    path = tuple([skeleton_points[i] for i in permutation])
    
    distance = calculate_distances(path)
    
    # code.interact(local=dict(globals(), **locals()))
    
    vectors_endpoints = [ (path[i+1],path[i]) for i in range(len(path) - 1) ]
    
    errors = [get_distance_to_nearest_vector(point, vectors_endpoints) for point in pcl_points]
    total_error = sum(errors)
    
    fitness = get_fitness(vertex_count-1, distance, total_error)
    
    # add back in the opposite path
    opposite_path = list(path)
    opposite_path.reverse()
    
    return ( (path, fitness), (tuple(opposite_path), fitness) )

PERMUTATIONS_FILE = "_permutations"
def get_paths(pool, skeleton_points, pcl_points, vertex_count):
    
    print "vertex_count:", vertex_count, "points:",len(skeleton_points)
    to_permute = len(skeleton_points)
    cache_file = PERMUTATIONS_FILE+"_"+str(vertex_count)+"_"+str(to_permute)+".marshal"
    permutations = []
    added_permutations = Set()
    
    if not os.path.exists(cache_file):
        indices = range(to_permute)
        indices_set = Set(indices)
        
        stack = [[_] for _ in indices]
    
        while len(stack) > 0:
            path = stack.pop()
        
            if len(path) == vertex_count:
                path = tuple(path)
                opposite = list(path)
                opposite.reverse()
                opposite = tuple(opposite)
                # only add one-direction of the path to the list, will handle later
                if not opposite in added_permutations:
                    permutations.append(path)
                    added_permutations.add(path)
                continue
        
            for next_index in indices_set - Set(path):
                new_path = copy.deepcopy(path)
                new_path.append(next_index)
                stack.append(new_path)
        marshal.dump(permutations, open(cache_file,'w'))
        print "\tsaved",cache_file
    else:
        permutations = marshal.load(open(cache_file, 'r'))
    
    combined = []
    
    # don't try using permutations generated with more vertices than you have, avoid indexing outside list
    inputs = [(vertex_count, permutation,skeleton_points, pcl_points)  for permutation in permutations if max(permutation) < to_permute]
    
    computed = pool.map(get_permutation_fitness, inputs)
    
    for path,opposite_path in computed:
        combined.append(path)
        combined.append(opposite_path)
        
    best_first = sorted(combined, key=lambda x:x[1], reverse=1)
    
    return best_first
    

if '__main__' == __name__:
    input_skeleton = sys.argv[1]
    input_pointcloud = sys.argv[2]

    pool = multiprocessing.Pool(NUM_THREADS)
    
    # somewhat arbitrary value for bounding concerns
    max_edges = 6

    pcl_validation_point_percentage = 0.2
    
    so_far = 0
    for skeleton_points, pcl_points in get_frames(input_skeleton, input_pointcloud):
        print SENTINEL,so_far
        so_far += 1
        sampled_pcl_points = random.sample(pcl_points, int(pcl_validation_point_percentage * len(pcl_points)))
        for edge_count in range(1,max_edges):
            if edge_count >= len(skeleton_points):
                continue
        
            permuted_paths = get_paths(pool, skeleton_points, sampled_pcl_points, edge_count+1)
            #code.interact(local=dict(globals(), **locals()))
            #break
        #break
        if so_far > 3:
            exit()
