import sys, random, math, code, os, multiprocessing, copy, time, gzip, popen2
import numpy as np
from sklearn.cluster import KMeans


OUTPUT_FOLDER = './clouds/'
if not os.path.exists(OUTPUT_FOLDER):
	os.mkdir(OUTPUT_FOLDER)

# units = centimeters
UNIT_SCALAR = 100

HEADER_DIVIDER = "$$$$$$$$$"
CAMERA_MARKER = '#Camera#'
SKELETON_MARKER = "#Skeleton#"
LENGTHS_HEADER = "#Lengths#"

RUNNING_ON_AWS = os.path.exists('./.on_aws')

MAX_CENTROIDS = 40

NUM_THREADS = int(sys.argv[sys.argv.index('-n') + 1]) if '-n' in sys.argv else 7

WRITE_OUT_CLOUD = False

LINK_COUNTS = [2,3,4,5,6]
MAX_LINKS = 8
PERMUTATIONS = 100
ROBOTS_PER_COUNT = 10

DENSITY_STEP = 0.1 * UNIT_SCALAR
DIMENSIONS = 3

LINK_LENGTHS = (10 * UNIT_SCALAR, 75 * UNIT_SCALAR)
LINK_STEP = 5 * UNIT_SCALAR
LINK_RADIUS = (2 * UNIT_SCALAR, 4 * UNIT_SCALAR)
WORKSPACE = 25 * UNIT_SCALAR
OFFSET_FROM_ORIGIN = WORKSPACE * 2

ALLOWED_CLOSENESS = LINK_LENGTHS[0] - 1 #points can't be closer than this

MIN_INTERIOR_ANGLE = math.pi / 12
MAX_INTERIOR_ANGLE = math.pi

CAMERA_LOCATION = (0, 0, 0)
CULL_OCCLUDED = True

ROTATION_THETA = math.pi / 20
UNSET, DECREASING, INCREASING = range(3)
DECREASING_STEP = -1 * ROTATION_THETA
INCREASING_STEP = ROTATION_THETA

EXTENSION, ROTATION  = range(2)
JOINT_TYPES = {EXTENSION : 'extension', }#ROTATION : 'rotation', }

def run_cmd(cmd):
	o,i = popen2.popen2(cmd)
	return o.read()


class Angle:
	def __init__(self, value = None):
		self.value = value if value else (random.random() * 0.6 * math.pi + 0.2 * math.pi)
		self.direction = UNSET
		self._set_direction()
	
	def step(self):
		return INCREASING_STEP if self.direction == INCREASING else DECREASING_STEP
	
	def _set_direction(self):
		max_cutoff = (7.0 / 8.0 * math.pi)
		min_cutoff = math.pi / 8
		# print "cutoffs:",min_cutoff, max_cutoff
		# print "was D:",('increasing' if self.direction == INCREASING else ('unset' if self.direction == UNSET else 'decreasing')),self.value
		
		if self.value <= min_cutoff or self.direction == UNSET:
			self.direction = INCREASING
		elif self.value >= max_cutoff:
			self.direction = DECREASING
		# print "now D:",('increasing' if self.direction == INCREASING else ('unset' if self.direction == UNSET else 'decreasing')),self.value
	
	def take_step(self):
		self.value += self.step()
		self._set_direction()
		return self.value

	def __repr__(self):
		return "<Angle @ "+str(self.value)+">"

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
	if not p0 or not p1:
		raise ValueError,"p0 or p1 not valid:"+str(p0)+","+str(p1)
	return tuple([p0[i] - p1[i] for i in range(len(p1))])

def distance_between(p0, p1):
	return vector_length(vector_between(p0, p1))

def angle_between_points(p0, p1, p2):
	# p1 is internal point
	norm_vec1 = normalize_vector(vector_between(p0, p1))
	norm_vec2 = normalize_vector(vector_between(p2, p1))
	radians = np.arccos(np.clip(np.dot(norm_vec1, norm_vec2), -1.0, 1.0))
	return radians

def point_plus_vector(p, v):
	return tuple([p[i] + v[i] for i in range(len(p))])

def get_theta():
	# radians
	return random.random() * 2 * math.pi

def line_between_points(p0, p1, step_size = 1, gen_cloud = False, link_radius = 0):
	data = []
	if not gen_cloud:
		data.append(p0)
	if not p1 or not p0:
		raise ValueError,"p0 or p1 not valid:"+str(p0)+","+str(p1)
	vector = vector_between(p1, p0)
	distance = vector_length(vector)
	step_vector = scale_vector(normalize_vector(vector), step_size)

	if gen_cloud:
		# from https://math.stackexchange.com/questions/731815/find-vector-find-on-3d-plane
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
			angle_between = angle_between_points(vertices[-2], last_vert, new_point)
			if angle_between < MIN_INTERIOR_ANGLE:
				new_point = None
				continue
			if too_close(vertices + [new_point]):
				new_point = None
				# print "Too close, throw away",tries
				tries -= 1
				continue
		vertices.append(tuple([round(x) for x in new_point]))
	return vertices

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

def gen_cloud(vertices, radii):
	edges = edges_between_vertices(vertices)
	edges_and_radii = []
	cloud = []
	for i,edge in enumerate(edges):
		p0,p1 = edge
		edges_and_radii.append( (p0,p1,radii[i]) )
	
	for p0,p1,radius in edges_and_radii:
		if not p0 or not p1:
			raise ValueError,"p0 or p1 not valid:"+str(p0)+","+str(p1)
		points = line_between_points(p0, p1, step_size = DENSITY_STEP, gen_cloud = True, link_radius = radius)
		
		if CULL_OCCLUDED:
			before = len(points)
			points = [point for point in points if is_visible(point, edges_and_radii)]
			
		cloud = cloud + points
	return cloud


def rotate_around_y(point, theta):
	x,y,z = point
	sin_theta = math.sin(theta)
	cos_theta = math.cos(theta)
	point = [
		# x
		z * sin_theta + x * cos_theta,
		# y
		y,
		# z
		z * cos_theta - x * sin_theta,
	]
	return tuple([round(x,3) for x in point])


def rotate_around_x(point, theta):
	x,y,z = point
	sin_theta = math.sin(theta)
	cos_theta = math.cos(theta)
	point = [
		# x
		x,
		# y
		y * cos_theta - z * sin_theta,
		# z
		y * sin_theta + z * cos_theta,
	]
	return tuple([round(x,3) for x in point])

def rotate_around_z(point, theta):
	x,y,z = point
	sin_theta = math.sin(theta)
	cos_theta = math.cos(theta)
	point = [
		# x
		x * cos_theta - y * sin_theta,
		# y
		x * sin_theta + y * cos_theta,
		# z
		z,
	]
	return tuple([round(x,3) for x in point])

def rotate_around_u(u, point, theta):
	# 2.3.3	Rotation matrix from axis and angle
	# https://en.wikipedia.org/wiki/Rotation_matrix#Rotation_matrix_from_axis_and_angle
	u_x, u_y, u_z = u
	p_x, p_y, p_z = point
	sin_theta = math.sin(theta)
	cos_theta = math.cos(theta)
	
	
	o_x = p_x * (cos_theta + (u_x ** 2 * (1.0 - cos_theta))) + \
			p_y * (u_x * u_y * (1.0 - cos_theta) - (u_z * sin_theta)) + \
			p_z * (u_x * u_z * (1.0 - cos_theta) + u_y * sin_theta)
	
	o_y = p_x * (u_y * u_x * (1.0 - cos_theta) + u_z * sin_theta) + \
			p_y * (cos_theta + (u_y**2) * (1.0 - cos_theta)) + \
			p_z * (u_y * u_z * (1.0 - cos_theta) - u_x * sin_theta)
	
	o_z = p_x * (u_z * u_x * (1.0 - cos_theta) - u_y * sin_theta) + \
			p_y * (u_z * u_y * (1.0 - cos_theta) + u_x * sin_theta) + \
			p_z * (cos_theta + (u_z ** 2) * (1 - cos_theta))
	
	return (o_x, o_y, o_z)
	
def get_axis_of_rotation_for(p0, p1, p2):
	# returns vector perpendicular to the plane containing (p0 - p1) and (p2 - p1)
	# represents the axis of rotation
	v0 = vector_between(p0, p1)
	v1 = vector_between(p2, p1)						
	return normalize_vector(np.cross(normalize_vector(v0), normalize_vector(v1)))
	

def compute_cloud(input):
	# random.seed(3)
	link_count, robot_i, link_lengths = input
	
	frames_per_vertex = 10
	frames_left = 0
	
	joint_types = [random.choice(JOINT_TYPES.keys()) for _ in range(link_count-1)]
	# print joint_types
	angles = [Angle() for _ in range(link_count-1)]
	
	radii = [random.randint(*LINK_RADIUS) for _ in range(link_count)]
	
	vertices = vertex_i = None
	
	padded_link_count = ('%0'+str(len(str(max(LINK_COUNTS))))+'d') % link_count
	padded_robot_i = ('%0'+str(len(str(ROBOTS_PER_COUNT)))+'d') % robot_i
	
	
	this_permutation_out = []
	outfile = "_".join([str(x) for x in [padded_link_count, padded_robot_i, int(time.time())]])+".txt"
	for permutation_i in range(PERMUTATIONS):
		# print "_______________"
		if permutation_i % 100 == 0:
			print link_count, permutation_i, int(time.time())
		
		
		if not vertices:
			# generate initial verts once
			while not vertices:
				vertices = gen_vertices(link_lengths)
		
		if len(vertices) == 0:
			print "No verts for some reason.",vertices
			break
		if vertices:
			# move some joints
			if frames_left == 0:
				vertex_i = random.choice(range(1,len(vertices) - 1))
				frames_left = frames_per_vertex
			# else:
			# 	print"using",vertex_i,frames_left
			frames_left -= 1
			this_vertex = vertices[vertex_i]
			angle_i = vertex_i - 1
			this_angle = angles[angle_i]
			
			axis_of_rotation = None
			this_joint_type = joint_types[angle_i]
			step = 0
			if EXTENSION == this_joint_type:
				this_angle.value = angle_between_points(vertices[vertex_i-1], this_vertex, vertices[vertex_i+1])
				step = this_angle.step()
				this_angle.take_step()
				axis_of_rotation = get_axis_of_rotation_for(vertices[vertex_i-1], this_vertex, vertices[vertex_i+1])
			elif ROTATION == this_joint_type:
				# semi-fictional steps, since no set starting angle. Results in it reversing periodically
				step = this_angle.step()
				this_angle.take_step()
				axis_of_rotation = vector_between(vertices[vertex_i-1], this_vertex)
			else:
				print "Unknown joint type"
				exit()
			
			
			
			# print ">",this_joint_type,axis_of_rotation
			# print e0,e1
			# print np.dot(v0, v1)
			# print np.dot(v0, axis_of_rotation)
			# print np.dot(v1, axis_of_rotation)
			# print this_angle.value
			# print angle_between_points(vertices[vertex_i-1], vertices[vertex_i], vertices[vertex_i+1])
			
			for move_vertex_i in range(vertex_i + 1, len(vertices)):
				
				vector_to_this_vertex = vector_between(vertices[move_vertex_i], this_vertex)
				rotated_vector = rotate_around_u(axis_of_rotation, vector_to_this_vertex, step)
				new_vertex = point_plus_vector(this_vertex, rotated_vector)
				# print "moving",move_vertex_i, vertices[move_vertex_i], vector_to_this_vertex, rotated_vector, new_vertex
				vertices[move_vertex_i] = new_vertex
		
		try:
			cloud = gen_cloud(vertices, radii)
		except: # ValueError:
			print "Something went wrong, skipping frame"
			continue
		
		# recompute joint_normals after rotation
		joint_normals = []
		for some_vertex_i in range(1, len(vertices) - 1):
			angle_i = some_vertex_i - 1
			axis_of_rotation = None
			this_joint_type = joint_types[angle_i]
			if EXTENSION == this_joint_type:
				axis_of_rotation = get_axis_of_rotation_for(vertices[some_vertex_i-1], vertices[some_vertex_i], vertices[some_vertex_i+1])
			elif ROTATION == this_joint_type:
				axis_of_rotation = vector_between(vertices[some_vertex_i-1], vertices[some_vertex_i])
			else:
				print "Unknown joint type"
				exit()
			joint_normals.append(tuple([round(x,2) for x in axis_of_rotation]))
		
		
		# remove camera, so the arm is zero-based
		start = vertices[0]
		camera_location = vector_between(CAMERA_LOCATION, start)
		shifted_vertices = [vector_between(vertex, start) for vertex in vertices]
		shifted_cloud = [vector_between(point, start) for point in cloud]
		
		
		padded_permutation_i = ('%0'+str(len(str(PERMUTATIONS)))+'d') % permutation_i
		
		this_permutation_out.append(HEADER_DIVIDER+"\n")
		this_permutation_out.append("#Perm#"+str(padded_permutation_i)+"\n")
		this_permutation_out.append(LENGTHS_HEADER+("\t".join([str(x) for x in link_lengths]))+"\n")
		this_permutation_out.append("#Vertices#")
		vert_out = []
		for vertex in shifted_vertices:
			vert_out.append(",".join([str(int(x)) for x in vertex]))
		this_permutation_out.append("\t".join(vert_out)+"\n")
		
		this_permutation_out.append("#JointTypes#"+("\t".join([JOINT_TYPES[x] for x in joint_types]))+"\n")
		
		normals_out = [",".join([str(x) for x in normal]) for normal in joint_normals]
		this_permutation_out.append("#JointNormals#"+("\t".join(normals_out))+"\n")
		
		# handle.write("#Angles#"+("\t".join([str(round(angle.value,3)) for angle in angles]))+"\n")
		
		this_permutation_out.append(CAMERA_MARKER+(",".join([str(x) for x in camera_location]))+"\n")
		
		try:
			skeleton = compute_skeleton(shifted_cloud, sum(link_lengths))
		except:
			print "invalid skeleton, skip a frame"
			continue
		
		to_write = [",".join([str(x) for x in point]) for point in skeleton]
		this_permutation_out.append(SKELETON_MARKER+"\t".join(to_write)+"\n")
		
		if WRITE_OUT_CLOUD:
			to_write = [",".join([str(x) for x in point]) for point in shifted_cloud]
			this_permutation_out.append("\n".join(to_write)+"\n")
		
	with gzip.GzipFile(OUTPUT_FOLDER+outfile+".gz",'a') as handle:
		handle.write("".join(this_permutation_out))
	
	if RUNNING_ON_AWS:
		cmd = "aws s3 --region us-east-1 cp "+OUTPUT_FOLDER+outfile+".gz s3://umbc.research/robot_learn_classifier/clouds/"
		print cmd
		run_cmd(cmd)
		run_cmd("rm "OUTPUT_FOLDER+outfile+".gz")
		

def get_skeleton_points(line):
	skeleton_text = line[len(SKELETON_MARKER):]
	skeleton_points = []
	for point in skeleton_text.split("\t"):
		point = [int(x) for x in point.split(",")]
		skeleton_points.append(point)
	return skeleton_points

def compute_skeleton(points, total_length):
	num_centroids = min(MAX_CENTROIDS, max(2, int(total_length / (10 * UNIT_SCALAR)))) # one per 10 'units'
	result = KMeans(n_clusters = num_centroids).fit(points)
	skeleton = []
	for point in result.cluster_centers_:
		skeleton.append( tuple([int(x) for x in point]))
	return skeleton

if '__main__' == __name__:
	inputs = []
	link_length_options = range(LINK_LENGTHS[0], LINK_LENGTHS[1], LINK_STEP)
	for link_count in LINK_COUNTS:
		for robot_i in range(ROBOTS_PER_COUNT):
			link_lengths = tuple([random.choice(link_length_options) for _ in range(link_count)])
			inputs.append( (link_count, robot_i, link_lengths) )
	inputs = list(set(inputs))
	inputs = sorted(inputs, reverse = True)
	print "Processing:",len(inputs)
#	for input in inputs:
#		print input
#	exit()

	# e0 = (10,0,0)
	# e1 = (10,0,20)
	# p0 = (5,0,0)
	# o = (0,0,0)
	#
	# print "+========="
	# print is_visible( p0, [ [e0, e1, 5 ] ] )
	# exit()
	
	if len(inputs) == 1 or NUM_THREADS == 1:
		[compute_cloud(x) for x in inputs]
	else:
		pool = multiprocessing.Pool(NUM_THREADS)
		pool.map(compute_cloud, inputs)
	# [compute_cloud(input) for input in inputs]


