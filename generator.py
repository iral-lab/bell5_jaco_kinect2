import sys, random, math, code, os, multiprocessing
import numpy as np

OUTPUT_FOLDER = './clouds/'
if not os.path.exists(OUTPUT_FOLDER):
	os.mkdir(OUTPUT_FOLDER)

# units = centimeters
UNIT_SCALAR = 100

LINK_COUNTS = [1,2,3,4,5,6]
VARIATIONS = 1
PERMUTATIONS = 1

DENSITY_STEP = 0.1 * UNIT_SCALAR

DIMENSIONS = 3
LINK_LENGTHS = (10 * UNIT_SCALAR, 75 * UNIT_SCALAR)
LINK_RADIUS = (2 * UNIT_SCALAR, 4 * UNIT_SCALAR)
WORKSPACE = 500 * UNIT_SCALAR
OFFSET_FROM_ORIGIN = WORKSPACE * 2

ALLOWED_CLOSENESS = LINK_LENGTHS[0] - 1 #points can't be closer than this

MIN_INTERIOR_ANGLE = 15
MAX_INTERIOR_ANGLE = 180

CAMERA_LOCATION = (0, 0, 0)
CULL_OCCLUDED = True


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
	start = tuple([int(random.random() * WORKSPACE + OFFSET_FROM_ORIGIN) for _ in range(DIMENSIONS)])
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
				# print "Too close, throw away",tries
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


# from https://stackoverflow.com/questions/2824478/shortest-distance-between-two-line-segments
def closestDistanceBetweenLines(a0,a1,b0,b1,clampAll=False,clampA0=False,clampA1=False,clampB0=False,clampB1=False):

	''' Given two lines defined by numpy.array pairs (a0,a1,b0,b1)
		Return the closest points on each segment and their distance
	'''

	# If clampAll=True, set all clamps to True
	if clampAll:
		clampA0=True
		clampA1=True
		clampB0=True
		clampB1=True

	# Calculate denomitator
	A = a1 - a0
	B = b1 - b0
	magA = np.linalg.norm(A)
	magB = np.linalg.norm(B)

	_A = A / magA
	_B = B / magB

	cross = np.cross(_A, _B);
	denom = np.linalg.norm(cross)**2

	# If lines are parallel (denom=0) test if lines overlap.
	# If they don't overlap then there is a closest point solution.
	# If they do overlap, there are infinite closest positions, but there is a closest distance
	if not denom:
		d0 = np.dot(_A,(b0-a0))
		# Overlap only possible with clamping
		if clampA0 or clampA1 or clampB0 or clampB1:
			d1 = np.dot(_A,(b1-a0))

			# Is segment B before A?
			if d0 <= 0 >= d1:
				if clampA0 and clampB1:
					if np.absolute(d0) < np.absolute(d1):
						return a0,b0,np.linalg.norm(a0-b0)
					return a0,b1,np.linalg.norm(a0-b1)


			# Is segment B after A?
			elif d0 >= magA <= d1:
				if clampA1 and clampB0:
					if np.absolute(d0) < np.absolute(d1):
						return a1,b0,np.linalg.norm(a1-b0)
					return a1,b1,np.linalg.norm(a1-b1)


		# Segments overlap, return distance between parallel segments
		return None,None,np.linalg.norm(((d0*_A)+a0)-b0)
	# Lines criss-cross: Calculate the projected closest points
	t = (b0 - a0);
	detA = np.linalg.det([t, _B, cross])
	detB = np.linalg.det([t, _A, cross])

	t0 = detA/denom;
	t1 = detB/denom;

	pA = a0 + (_A * t0) # Projected closest point on segment A
	pB = b0 + (_B * t1) # Projected closest point on segment B

	# Clamp projections
	if clampA0 or clampA1 or clampB0 or clampB1:
		if clampA0 and t0 < 0:
			pA = a0
		elif clampA1 and t0 > magA:
			pA = a1

		if clampB0 and t1 < 0:
			pB = b0
		elif clampB1 and t1 > magB:
			pB = b1

		# Clamp projection A
		if (clampA0 and t0 < 0) or (clampA1 and t0 > magA):
			dot = np.dot(_B,(pA-b0))
			if clampB0 and dot < 0:
				dot = 0
			elif clampB1 and dot > magB:
				dot = magB
			pB = b0 + (_B * dot)

		# Clamp projection B
		if (clampB0 and t1 < 0) or (clampB1 and t1 > magB):
			dot = np.dot(_A,(pB-a0))
			if clampA0 and dot < 0:
				dot = 0
			elif clampA1 and dot > magA:
				dot = magA
			pA = a0 + (_A * dot)
	return pA,pB,np.linalg.norm(pA-pB)



def distance_between_segments(segment_1, segment_2):
	args = [segment_1[0], segment_1[1], segment_2[0], segment_2[1]]
	args = [np.array(x) for x in args]
	pa,pb,distance = closestDistanceBetweenLines(args[0], args[1], args[2], args[3], clampAll = True)
	if distance or distance == 0:
		return distance
	raise ValueError,"unknown distance between "+str(segment_1)+", "+str(segment_2)

def is_visible(point, edges):
	to_camera = (point, CAMERA_LOCATION)
	
	for p0, p1, radius in edges:
		link_segment = (p0, p1)
		distance = distance_between_segments(link_segment, to_camera)
		# print link_segment
		# print to_camera
		# print distance
		# print radius
		
		if radius > distance:
			return False
	return True

def gen_cloud(vertices):
	edges = edges_between_vertices(vertices)
	edges_and_radii = []
	cloud = []
	for p0,p1 in edges:
		radius = random.randint(*LINK_RADIUS)
		edges_and_radii.append( (p0,p1,radius) )
	
	for p0,p1,radius in edges_and_radii:
		points = line_between_points(p0, p1, step_size = DENSITY_STEP, gen_cloud = True, link_radius = radius)
		
		if CULL_OCCLUDED:
			before = len(points)
			points = [point for point in points if is_visible(point, edges_and_radii)]
			
		cloud = cloud + points
	return cloud

def compute_cloud(input):
	# random.seed(3)
	link_count, variation_i, link_lengths = input
	
	vertices = angles = None
	for permutation_i in range(PERMUTATIONS):
		print link_count, variation_i, permutation_i

		if not vertices:
			# generate initial verts/angles once
			while not vertices:
				vertices, angles = gen_vertices(link_lengths)
		else:
			# move some joints
			valid = False
			while not valid:
				vertex_i = random.choice(range(len(vertices) - 2)) + 1
				angle_i = angles[vertex_i - 1]
				
				slight_move = math.pi / 12 # 15 degrees
				
				# need transformation matrix to rotate vertices above angle_i around vertex_i
				#in_plane_vector = vector_between(vertices[vertex_i-1], vertices[vertex_i])
				#perpendicular_u = perpendicular_vector(in_plane_vector)
				
				
				
				# safety break
				break
		
		
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
			handle.write("#Camera#"+(",".join([str(x) for x in CAMERA_LOCATION]))+"\n")
			to_write = [",".join([str(x) for x in point]) for point in cloud]
			handle.write("\n".join(to_write))

if '__main__' == __name__:
	inputs = []
	for link_count in LINK_COUNTS:
		for variation_i in xrange(VARIATIONS):
			link_lengths = [random.randint(*LINK_LENGTHS) for _ in range(link_count)]
			inputs.append( (link_count, variation_i, link_lengths) )
	inputs.reverse()
	
	# e0 = (10,0,0)
	# e1 = (10,0,20)
	# p0 = (5,0,0)
	# o = (0,0,0)
	#
	# print "+========="
	# print is_visible( p0, [ [e0, e1, 5 ] ] )
	# exit()
	
	if len(inputs) == 1:
		compute_cloud(inputs[0])
	else:
		pool = multiprocessing.Pool(4)
		pool.map(compute_cloud, inputs)
	# [compute_cloud(input) for input in inputs]


