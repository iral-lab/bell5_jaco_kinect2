import sys, random, math, code, os, multiprocessing
import numpy as np

OUTPUT_FOLDER = './clouds/'
if not os.path.exists(OUTPUT_FOLDER):
	os.mkdir(OUTPUT_FOLDER)

# units = millimeters

LINK_COUNTS = [2,3,4,5,6]
VARIATIONS = 1
PERMUTATIONS = 1

DENSITY_STEP = 1.0

DIMENSIONS = 3
LINK_LENGTHS = (100, 500)
LINK_RADIUS = (20, 40)
WORKSPACE = 10000

ALLOWED_CLOSENESS = LINK_LENGTHS[0] - 1 #points can't be closer than this

MIN_INTERIOR_ANGLE = 15
MAX_INTERIOR_ANGLE = 180

def vector_length(v):
	return math.sqrt(sum([v[i]*v[i] for i in range(DIMENSIONS)]))

def normalize_vector(v):
	length = vector_length(v)
	if 0 == length:
		return tuple([0] * DIMENSIONS)
	return tuple([x / length for x in v])

def get_some_unit_vector():
	return tuple(normalize_vector([random.random() - 0.5 for _ in range(DIMENSIONS)]))

def scale_vector(v, s):
	return tuple([x * s for x in v])

def vector_between(p0, p1):
	return tuple([p0[i] - p1[i] for i in range(len(p1))])

def distance_between(p0, p1):
	return vector_length(vector_between(p0, p1))

def angle(p0, p1, p2):
	# p1 is internal point
	norm_vec1 = normalize_vector(vector_between(p0, p1))
	norm_vec2 = normalize_vector(vector_between(p2, p1))
	radians = np.arccos(np.clip(np.dot(norm_vec1, norm_vec2), -1.0, 1.0))
	return radians * 180 / math.pi

def point_plus_vector(p, v):
	return tuple([p[i] + v[i] for i in range(len(p))])

def get_theta():
	# radians
	return random.random() * 2 * math.pi

def line_between_points(p0, p1, step_size = 1, gen_cloud = False, link_radius = 0):
	data = []
	if not gen_cloud:
		data.append(p0)
	vector = vector_between(p1, p0)
	distance = vector_length(vector)
	step_vector = scale_vector(normalize_vector(vector), step_size)

	if gen_cloud:
		# from https://math.stackexchange.com/questions/731815/find-vector-rotated-on-3d-plane
		perp_vector = normalize_vector(perpendicular_vector(vector)) #e1
		perp_cross_vector = np.cross(vector, perp_vector) # p
		cross_norm = normalize_vector(perp_cross_vector) # e2

	t = step_size
	last_point = p0
	while t < distance:
		new_point = point_plus_vector(last_point, step_vector)
		if gen_cloud:
			theta = get_theta()
			cos_vec = tuple([math.cos(theta) * perp_vector[i] for i in range(DIMENSIONS)])
			sin_vec = tuple([math.sin(theta) * cross_norm[i] for i in range(DIMENSIONS)])
			rotated_vec = tuple([link_radius * (cos_vec[i] + sin_vec[i]) for i in range(DIMENSIONS)])
			cloud_point = point_plus_vector(new_point, rotated_vec)
			data.append(tuple([int(round(x)) for x in cloud_point]))
		else:
			data.append(new_point)
		t += step_size
		last_point = new_point
	if not gen_cloud:
		data.append(p1)
	return data

def edges_between_vertices(vertices):
	return [(vertices[i], vertices[i+1]) for i in range(len(vertices)-1)]

def too_close(vertices):
	edges = edges_between_vertices(vertices)
	lines = [line_between_points(p0,p1) for p0,p1 in edges]
	num_lines = len(lines)
	for i in range(num_lines):
		for j in range(i+1, num_lines):
			if abs(j-i) <= 1:
				continue
			edge_i = edges[i]
			edge_j = edges[j]
			for p_i in edge_i:
				for p_j in edge_j:
					if distance_between(p_i, p_j) < ALLOWED_CLOSENESS:
						return True
	return False

def gen_vertices(link_lengths):
	vertices = []
	start = tuple([int(random.random() * WORKSPACE) for _ in range(DIMENSIONS)])
	vertices.append(start)
	angles = []
	for link_i, length in enumerate(link_lengths):
		last_vert = vertices[-1]
		new_point = None
		tries = 100
		while not new_point:
			if tries <= 0:
				print "Resetting"
				return (None, None)
			direction = get_some_unit_vector()
			distance_vector = scale_vector(direction, length)
			new_point = point_plus_vector(last_vert, distance_vector)
			if 0 == link_i:
				break
			assert(len(vertices) > 1)
			angle_between = angle(vertices[-2], last_vert, new_point)
			if angle_between < MIN_INTERIOR_ANGLE:
				new_point = None
				continue
			if too_close(vertices + [new_point]):
				new_point = None
				print "Too close, throw away",tries
				tries -= 1
				continue
			angles.append(round(angle_between,2))
		vertices.append(tuple([round(x) for x in new_point]))
	return (vertices, angles)

def perpendicular_vector(v):
	if 0 == v[0] and 0 == v[1]:
		if 0 == v[2]:
			raise ValueError("Zero vector")
		return (0,1,0)
	return (-1 * v[1], v[0], 0)

def gen_cloud(vertices):
	edges = edges_between_vertices(vertices)
	cloud = []
	for p0,p1 in edges:
		points = line_between_points(p0, p1, step_size = DENSITY_STEP, gen_cloud = True, link_radius = random.randint(*LINK_RADIUS))
		cloud = cloud + points
	return cloud

def compute_cloud(input):
	link_count, variation_i, link_lengths, permutation_i = input
	print link_count, variation_i, permutation_i

	vertices = angles = None
	while not vertices:
		vertices, angles = gen_vertices(link_lengths)
	cloud = gen_cloud(vertices)

	outfile = "_".join([str(x) for x in [link_count, variation_i, permutation_i]])+".txt"
	with open(OUTPUT_FOLDER+outfile,'w') as handle:
		handle.write("#Lengths#"+("\t".join([str(x) for x in link_lengths]))+"\n")
		handle.write("#Vertices#")
		vert_out = []
		for vertex in vertices:
			vert_out.append(",".join([str(int(x)) for x in vertex]))
		handle.write("\t".join(vert_out)+"\n")
		handle.write("#Angles#"+("\t".join([str(x) for x in angles]))+"\n")
		to_write = [",".join([str(x) for x in point]) for point in cloud]
		handle.write("\n".join(to_write))

if '__main__' == __name__:
	inputs = []
	for link_count in LINK_COUNTS:
		for variation_i in xrange(VARIATIONS):
			link_lengths = [random.randint(*LINK_LENGTHS) for _ in range(link_count)]
			for permutation_i in xrange(PERMUTATIONS):
				inputs.append( (link_count, variation_i, link_lengths, permutation_i) )
	inputs.reverse()
	pool = multiprocessing.Pool(4)
	pool.map(compute_cloud, inputs)
	# [compute_cloud(input) for input in inputs]


