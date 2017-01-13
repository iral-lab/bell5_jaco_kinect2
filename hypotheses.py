import sys, itertools, copy, marshal, os, code, multiprocessing
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

def euclid_distance(p1,p2):
    return np.linalg.norm(np.array(p1) - np.array(p2))

def calculate_distances(points):
    sum_distance = 0.0
    for i in range(len(points) - 1):
        start,stop = points[i:i+2]
        sum_distance += euclid_distance(start,stop)
    return sum_distance

PERMUTATIONS_FILE = "_permutations"
def get_paths(pool, points, vertex_count):
    print "vertex_count:", vertex_count, "points:",len(points)
    to_permute = len(points)
    cache_file = PERMUTATIONS_FILE+"_"+str(vertex_count)+"_"+str(to_permute)+".marshal"
    permutations = []
    if not os.path.exists(cache_file):
        indices = range(to_permute)
        indices_set = Set(indices)
        
        stack = [[_] for _ in indices]
    
        while len(stack) > 0:
            path = stack.pop()
        
            if len(path) == vertex_count:
                permutations.append(path)
                continue
        
            for next_index in indices_set - Set(path):
                new_path = copy.deepcopy(path)
                new_path.append(next_index)
                stack.append(new_path)
        marshal.dump(permutations, open(cache_file,'w'))
        print "\tsaved",cache_file
    else:
        permutations = marshal.load(open(cache_file, 'r'))
    
    #print len(permutations)
    #code.interact(local=dict(globals(), **locals()))
    
    ordered_points = []
    for permutation in permutations:
        if max(permutation) >= len(points):
            # to account for over-provisioned cached permutations
            continue
    
        ordered_points.append( tuple([points[i] for i in permutation]) )
   
    #print "ordered points:",len(ordered_points)
    #code.interact(local=dict(globals(), **locals()))
    
    distances = pool.map(calculate_distances, ordered_points)
    
    combined = [ (ordered_points[i], distance) for i,distance in enumerate(distances)]
    best_first = sorted(combined, key=lambda x:x[1])
    
    #code.interact(local=dict(globals(), **locals()))
    #print ordered_points


if '__main__' == __name__:
    input_skeleton = sys.argv[1]
    input_pointcloud = sys.argv[2]

    pool = multiprocessing.Pool(NUM_THREADS)
    
    # somewhat arbitrary value for bounding concerns
    max_edges = 6

    so_far = 0
    for frame_points in csv_reader(input_skeleton):
        print SENTINEL,so_far
        so_far += 1
        for edge_count in range(1,max_edges):
            if edge_count >= len(frame_points):
                continue
        
            permuted_paths = get_paths(pool, frame_points, edge_count+1)
            #break
        #break
