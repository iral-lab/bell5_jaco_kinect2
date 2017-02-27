import sys, multiprocessing, cPickle, time, random, hashlib, code, copy, math
from sets import Set
from hypotheses import csv_reader, CACHE_FOLDER, SENTINEL, get_frames, euclid_distance, round_to_precision, vector_between, length_3d

NUM_THREADS = int(sys.argv[sys.argv.index('-t')+1]) if '-t' in sys.argv else 8
PRECISION_DIGITS = 3

COMPRESSION = cPickle
COMPRESSION_EXTENSION = '.pickle'

MAX_EDGES = 6


def get_ordered_nearest_points(skeleton_points):
	skeleton_distances = {}
	ordered_skeleton_near_points = {}
	num_points = len(skeleton_points)
	for i in range(num_points):
		p0 = skeleton_points[i]
		for j in range(i+1, num_points):
			p1 = skeleton_points[j]
			distance = euclid_distance(p0, p1)
			
			if not p0 in skeleton_distances:
				skeleton_distances[p0] = []
			skeleton_distances[p0].append( (distance, p1) )
			
			if not p1 in skeleton_distances:
				skeleton_distances[p1] = []
			skeleton_distances[p1].append( (distance, p0) )
		
		distance_point_pairs = sorted(skeleton_distances[p0])
		# keep track of full list, don't be strict, as it might truncate if num_closest points are isolated
		ordered_skeleton_near_points[p0] = [point for distance,point in distance_point_pairs]
	return ordered_skeleton_near_points

def get_num_closest(num_points):
	return min(5, num_points)

def get_paths(pool, skeleton_points, sampled_pcl_points, vertex_count):
   
	#skeleton_points = [(1,1,1),(2,2,2),(3,3,3),(4,4,4),(5,5,5)]
	num_closest = get_num_closest(len(skeleton_points))
	
	ordered_skeleton_near_points = get_ordered_nearest_points(skeleton_points)
	
	# only let the n points with lowest y-axis be "anchor" points
	possible_anchors = get_anchors(skeleton_points)
	
	stack = []
	for point in possible_anchors:
		stack.append( [point] )
	
	paths = []
	
	while len(stack) > 0:
		path = stack.pop()
		
		if len(path) == vertex_count:
			paths.append(path)
			continue
		
		last_point = path[-1]
		added = 0
		
		for nearest in ordered_skeleton_near_points[last_point]:
			if added >= num_closest:
				break
			if nearest in path:
				continue
			new_path = copy.deepcopy(path)
			new_path.append(nearest)
			stack.append(new_path)
			added += 1
	
	return paths

def get_anchors(possibles):
	lowest_y_anchors = min(3, len(possibles))
	y_index = 1
	anchors = [(point[y_index], point) for point in possibles]
	sorted_anchors = sorted(anchors)[:lowest_y_anchors]
	return [point for y,point in sorted_anchors]

def path_to_candidate(path):
	lengths = []
	for i in range(len(path)-1):
		lengths.append( round( euclid_distance(path[i], path[i+1]), PRECISION_DIGITS) )
	return lengths

def normalize_vector(vector):
	l = length_3d(vector)
	return tuple([p/l for p in vector])

def get_ordered_other_points(source, points):
	distances = [(euclid_distance(p, source), p) for p in points]
	sorted_distance = sorted(distances)
	return [p[1] for p in sorted_distance]

def score_candidate_against_frame(candidate, skeleton_points, pcl_points):
	anchors = get_anchors(skeleton_points)
	
	
	num_closest = get_num_closest(len(skeleton_points))
	
	stack = []
	for anchor in anchors:
		stack.append( [anchor] )
	
	final_paths = []
	
	max_path_length = len(candidate) + 1
	
	while len(stack) > 0:
		path = stack.pop()
		
		if len(path) == max_path_length:
			final_paths.append(path)
			continue

		previous = path[-1]
		
		# get a new list of possible "nearby" points, since our attachment will definitely result in new points
		path_points = Set(path)
		valid_points = [p for p in skeleton_points if not p in path_points]		
		nearby_points = get_ordered_other_points(previous, valid_points)
		
		# iterate through the nearest points
		added = 0
		for next_point in nearby_points:
			if added >= num_closest:
				break
			elif next_point in path:
				continue
			
			added += 1
			
			vector = vector_between(next_point, previous)
			normalized = normalize_vector(vector)
			
			link_length = candidate[ len(path) - 1]
			
			movement_vector = tuple([link_length * val for val in normalized])
			
			this_endpoint = tuple([round(previous[i] + movement_vector[i], PRECISION_DIGITS) for i in range(len(movement_vector))])
			
			new = copy.deepcopy(path)
			new.append(this_endpoint)
			stack.append( new )
	
	
	
	scored = [(score_path_against_points(path, pcl_points), path) for path in final_paths]
	
	temp_best = sorted(scored, reverse=True)
	score, best_path = temp_best[0]
	
	# code.interact(local=dict(globals(), **locals()))
	
	
	return score

def v_dot(u,v):
	return (u[0] * v[0] + u[1] * v[1] + u[2] * v[2])
def v_norm(v):
	return math.sqrt(v_dot(v,v))
def v_dist(u,v):
	diff = vector_between(u,v)
	return v_norm(diff)
	
def dist_to_segment(p, s0, s1):
	# from dist_Point_to_Segment() http://geomalgorithms.com/a02-_lines.html
	v = vector_between(s1, s0)
	w = vector_between(p, s0)
	c1 = v_dot(w,v)
	if c1 < 0:
		return v_dist(p, s0)
	c2 = v_dot(v,v)
	if c2 <= c1:
		return v_dist(p, s1)
	
	b = c1 / c2
	pb = tuple([s0[i] + b * v[i] for i in range(len(p))])
	return v_dist(p, pb)
	
		

def score_path_against_points(path, pcl_points):
	if len(path) == 0:
		return 0.0
	
	endpoints = [(path[i], path[i+1]) for i in range(len(path) - 1)]
	
	total = 0.0
	for point in pcl_points:
		min_dist = 9999999.0
		for p0,p1 in endpoints:
			dist = dist_to_segment(point, p0, p1)
			min_dist = min(min_dist, dist)
		total += min_dist
	
	return -1 * total

def get_candidates(pool, skeleton_points, sampled_pcl_points, vertex_count):
	paths = get_paths(pool, skeleton_points, sampled_pcl_points, vertex_count)
	
	candidates = [tuple(path_to_candidate(path)) for path in paths]
	
	return candidates
	# won't generally be done here, but we are for testing/dev
	#score = score_candidate_against_frame(candidates[0], skeleton_points, sampled_pcl_points)
	
	#print candidates[0], score
	
	#code.interact(local=dict(globals(), **locals()))
	#return paths

def do_analysis():

	input_skeleton = sys.argv[sys.argv.index('-s')+1]
	input_pointcloud = sys.argv[sys.argv.index('-p')+1]
	
	pool = multiprocessing.Pool(NUM_THREADS)
		
	start_id = str(int(time.time()))
	
	best_case_output_file = "attachment_best_values_"+start_id+".csv"
	
	columns = ["Frame"] + [str(x) for x in range(1, MAX_EDGES+1)]
	#open(best_case_output_file,'w').write("\t".join(columns)+"\n")
	
	best_path_output = "attachment_best_path_"+start_id+".csv"
	open(best_path_output, 'w').write('Frame\tPath\n')
	
	max_points_to_use = 500
	
	previous_frames = []
	
	candidates_so_far = Set()
	candidate_frames_to_compute = multiprocessing.Queue()
	
	#random.seed(1)	
	frame_number = 0
	for skeleton_points, pcl_points in get_frames(input_skeleton, input_pointcloud):
		frame_start = time.time()
		print SENTINEL,"frame:",frame_number

		# code.interact(local=dict(globals(), **locals()))
		
		points_to_use = min(max_points_to_use, len(pcl_points))
		sampled_pcl_points = random.sample(pcl_points, points_to_use)
		print "Using",len(sampled_pcl_points),"of",len(pcl_points),"Pointcloud points"
		
		skeleton_points = [round_to_precision(point, PRECISION_DIGITS) for point in skeleton_points]
		sampled_pcl_points = [round_to_precision(point, PRECISION_DIGITS) for point in sampled_pcl_points]
		
		bests = []
		
		for edge_count in range(2,MAX_EDGES+1):
			# go in reverse order, most -> least
			# edge_count = (MAX_EDGES+1) - edge_count

			if edge_count >= len(skeleton_points):
				continue
			
			input_tuple = (skeleton_points, pcl_points, edge_count)
			input_hash = hashlib.md5(str(input_tuple)).hexdigest()
			cache_file = CACHE_FOLDER+"_attachment_frame_"+input_hash+COMPRESSION_EXTENSION
			candidates = None
			vertex_count = edge_count+1
			
			print "vertex_count:", vertex_count, "points:",len(skeleton_points)
			
			cache_file_exists = False #os.path.exists(cache_file)
			
			if cache_file_exists:
				try:
					print "\tLOADING CACHE",cache_file
					candidates = COMPRESSION.loads(open(cache_file,'rb').read())
					print "\tdone"
				except ValueError:
					print "Caught COMPRESSION error, bugged file:",cache_file
					candidates = None
			
			else:
				candidates = get_candidates(pool, skeleton_points, sampled_pcl_points, vertex_count)
				open(cache_file,'wb').write(COMPRESSION.dumps(candidates))
				print "\tSaved paths to",cache_file	
			
			if not candidates:
				print "\tno candidates, quitting"
				exit()
			
			previous_frames.append( (frame_number, skeleton_points, sampled_pcl_points) )
			
			# now cross with all previous frames.
			# inclusive with current frame due to adding it to previous frames as above
			for candidate in candidates:
				for old_frame_number, old_skeleton_points, old_sampled_pcl_points in previous_frames:
					key = (candidate, old_frame_number)
					if key in candidates_so_far:
						continue
					candidates_so_far.add(key)
					candidate_frames_to_compute.put( (candidate, old_frame_number, old_skeleton_points, old_sampled_pcl_points) )
			print "\tdone pushing onto computation queue",len(candidates_so_far)
			#code.interact(local=dict(globals(), **locals()))
			
		#open(best_case_output_file, 'a').write("\t".join([str(frame_number)] + [str(x) for x in bests])+"\n")
		print ">> frame took",round(time.time() - frame_start, 2),"seconds"
		frame_number += 1
	

if '__main__' == __name__:

	if len(sys.argv) < 4 or not '-s' in sys.argv or not '-p' in sys.argv:
		print "Usage: python",sys.argv[0]," <-t num-threads> -s skeleton_file.csv -p pointcloud.csv"
		print "example: python",sys.argv[0],"-s datasets/diverse_movement_skeleton.csv  -p datasets/diverse_movement_pcl.csv"
		exit()

	do_analysis()

