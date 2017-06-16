import sys, json, multiprocessing, time, random, code, math, os
import numpy as np

# code.interact(local=dict(globals(), **locals())) 

SCALE = 1

TERMINATOR = "terminate"
DRAW_DELAY = 0.3
MAX_POINTS = 1000

BLUE = [ 0.20815755, 0.4907831, 0.72991901, 1]
RED = [ 0.9135442  , 0.48970524 , 0.56584265 , 1]
GREEN = [ 0.34262711 , 0.75813294 , 0.34156955 , 1]
BLACK = [0,0,0,1]
WHITE = [1,1,1,1]

EDGE_COUNT_OVERRIDE = False

CLUSTER_COLOR = RED
SKELETON_COLOR = BLUE
LINE_COLOR = GREEN # np.random.uniform(0,1,4)

POINT = "point"
LINE = "line"

BIG = 10
SMALL = 5

BLACK_ON_WHITE = True


from glumpy import app
from glumpy.graphics.collections import PointCollection, SegmentCollection
from glumpy.graphics.collections import PathCollection

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
				batch.append( tuple([float(x) * SCALE for x in line.split(",")]) )
		yield batch

def get_frames(skeleton_csv, pcl_csv, paths_csv):
	skeleton_reader = csv_reader(skeleton_csv)
	pcl_reader = csv_reader(pcl_csv)
	path_reader = csv_reader(paths_csv)
	
	
	skeleton_frame = skeleton_reader.next()
	pcl_frame = pcl_reader.next()
	path_frame = path_reader.next()
	
	while skeleton_frame or pcl_frame or path_frame:
		frame = (skeleton_frame, pcl_frame, path_frame)
		yield frame
		
		skeleton_frame = skeleton_reader.next()
		pcl_frame = pcl_reader.next()
		path_frame = path_reader.next()

FRAME_N = 0
DIM = 1536
def start_visualizing(cluster_points, frame_to_show):
	window = app.Window(DIM,DIM, color=(BLACK if BLACK_ON_WHITE else (1,2,1,1)))
	point_collection = PointCollection("agg", color="local", size="local")
	paths = PathCollection(mode="agg")
	
	@window.event
	def on_draw(dt):
		global FRAME_N
		window.clear()
		point_collection.draw()
		
		if not cluster_points.empty():
			possible = cluster_points.get()
			
			print "frame",FRAME_N
			FRAME_N += 1
			if frame_to_show > -1 and not (FRAME_N-1) == frame_to_show:
				while not possible == TERMINATOR:
					possible = cluster_points.get()
				return
			time.sleep(DRAW_DELAY)
		
			while len(point_collection) > 0:
				del point_collection[0]
			
			while len(paths) > 0:
				del paths[0]
			
			while not possible == TERMINATOR:
				type, data, color, size = possible
				if POINT == type:
					new_point = [[0.7*x for x in data]]
					new_point[0][1] *= -1
					point_collection.append(new_point,
									  color = color,
									  size  = size)
				elif LINE == type:
					path = []
					for hop in data:
						new_hop = [round(0.7 * abs(x),5) for x in hop]
						path.append(new_hop)
					path = np.asarray(path)
					print path, len(path)
					paths.append(path, closed=True, itemsize=len(path))
					paths["linewidth"] = 3.0
				
				possible = cluster_points.get()
	


	window.attach(point_collection["transform"])
	window.attach(point_collection["viewport"])
	app.run()


def euclid_distance(p1,p2):
	return np.linalg.norm(np.array(p1) - np.array(p2))

def generate_line(p0, p1):
	density = 0.005
	
	distance = euclid_distance(p0,p1)
	
	delta = round(density / distance, 3)
	
	t = delta
	v = []
	for i in range(3):
		v.append( round(p0[i] - p1[i],6) )
	v = tuple(v)
	line = []
	
	
	while t < 1:
		p = []
		
		for i in range(3):
			p.append( round(p1[i] + t * v[i], 6))
		
		line.append(tuple(p))
		
		t += delta
	return line
	

def process_files(skeleton_csv, pcl_csv, best_frame_csv, cluster_points):
	frame_n = 0
	for skeleton_frame, pcl_frame, path_frame in get_frames(skeleton_csv, pcl_csv, best_frame_csv):
		frame_n += 1
		to_render = pcl_frame
		
		if len(to_render) > MAX_POINTS:
			to_render = random.sample(to_render, MAX_POINTS)
		min_v = 99999
		max_v = -99999
		v_ind = 1
		for point in to_render:
			min_v = min(point[v_ind], min_v)
			max_v = max(point[v_ind], max_v)
		
		for point in to_render:
			n = float((point[v_ind] + abs(min_v) * 1.0) / (max_v+ abs(min_v)))
			this_color = [ n, n, n, 1]
			this_color = WHITE if BLACK_ON_WHITE else BLACK
			cluster_points.put( (POINT, point, this_color, BIG) )

		for point in skeleton_frame:
			cluster_points.put( (POINT, point, SKELETON_COLOR, BIG) )
		
		for point in path_frame:
			cluster_points.put( (POINT, point, LINE_COLOR, BIG) )
			print point
		
		
		endpoints = [(path_frame[i], path_frame[i+1]) for i in range(len(path_frame)-1)]
		for line in [generate_line(p0,p1) for p0,p1 in endpoints]:
			for point in line:
				cluster_points.put( (POINT, point, LINE_COLOR, SMALL) )
		
		cluster_points.put(TERMINATOR)
	print "broke"

def vector_length(v):
	return math.sqrt(sum([v[i] * v[i] for i in range(len(v))]))

def rotate_around_y(point, theta):
	x,y,z = point
	point = [
		# x
		z * math.sin(theta) + x * math.cos(theta),
		# y
		y,
		# z
		z * math.cos(theta) - x * math.sin(theta),
	]
	return point

def generator_render(cloud_file, cluster_point_queue):
	# (-0.137504, -0.407314, 1.117)
	this_color = WHITE if BLACK_ON_WHITE else BLACK
	
	if not os.path.exists(cloud_file):
		print "File doesn't exist:", cloud_file
		return
	
	points = []
	
	average_point = [0.0] * 3
	for line in open(cloud_file,'r').readlines():
		line = line.strip()
		if '' == line or '#' in line:
			continue
		
		point = tuple([float(x) for x in line.split(',')])
		points.append(point)
		average_point = [average_point[i] + point[i] for i in range(len(point))]
	
	average_point = [round(average_point[i] / len(points), 5) for i in range(len(average_point))]
	max_length = 0.0
	for i,point in enumerate(points):
		points[i] = [point[j] - average_point[j] for j in range(len(point))]
		max_length = max(max_length, vector_length(points[i]))

	for i,point in enumerate(points):
		point = [point[j] / max_length for j in range(len(point))]
		points[i] = tuple([round(point[j], 5) for j in range(len(point))])
	
	
	# radians
	theta = math.pi / 16
	v_ind = 2
	while True:
		for i,point in enumerate(points):
			points[i] = rotate_around_y(point, theta)
		
		points = sorted(points, key = lambda x:x[v_ind])
		
		for point in points:
			n = float(point[v_ind] + 1.3 / 2 )
			this_color = [ n, n, n, 1]
			cluster_point_queue.put( (POINT, point, this_color, SMALL) )
		
		cluster_point_queue.put(TERMINATOR)
	
	# code.interact(local=dict(globals(), **locals())) 
	
	pass


if '__main__' == __name__:
	GENERATOR_RENDER = True
	
	if GENERATOR_RENDER and len(sys.argv) < 2:
		print "python",sys.argv[0],"clouds/file.txt"
		exit()
		
	elif not GENERATOR_RENDER and len(sys.argv) < 3:
		print "python",sys.argv[0],"skeleton.csv pcl.csv best_frame_robots_[i]"
		exit()
	
	cluster_points = multiprocessing.Queue()
	frame_to_show = -1
	
	if GENERATOR_RENDER:
		cloud_file = sys.argv[1]
		processor = multiprocessing.Process(target = generator_render, args = (cloud_file, cluster_points))
		processor.start()
	else:
		skeleton_csv, pcl_csv, best_frame_robot = sys.argv[1:4]
		frame_to_show = int(sys.argv[4]) if len(sys.argv) > 4 else -1	
		processor = multiprocessing.Process(target = process_files, args = (skeleton_csv, pcl_csv, best_frame_robot, cluster_points))
		processor.start()
	
	start_visualizing(cluster_points, frame_to_show)
	
	
	
	
	
	
