import sys, json, multiprocessing, time, random, code
import numpy as np

# code.interact(local=dict(globals(), **locals())) 

SCALE = 1

TERMINATOR = "terminate"
DRAW_DELAY = 1.0
MAX_POINTS = 1000

BLUE = [ 0.20815755, 0.4907831, 0.72991901, 1]
RED = [ 0.9135442  , 0.48970524 , 0.56584265 , 1]
GREEN = [ 0.34262711 , 0.75813294 , 0.34156955 , 1]

EDGE_COUNT_OVERRIDE = False

CLUSTER_COLOR = RED
SKELETON_COLOR = BLUE
LINE_COLOR = GREEN # np.random.uniform(0,1,4)

POINT = "point"
LINE = "line"

BIG = 10
SMALL = 5


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
		if len(batch) > 0:
			yield batch


def paths_reader(input_file):
	with open(input_file, 'r') as handle:
		next_line = handle.readline()
		# skip first line header
		next_line = handle.readline()
		batch = {}
		last_frame = None
		while next_line:
			line = next_line.strip()
			next_line = handle.readline()
			parts = line.split("\t")
			frame, edge_count = parts[0:2]
			path = parts[2:]
			frame = int(frame)
			edge_count = int(edge_count)
			
			if last_frame and not frame == last_frame:
				yield batch
				batch = {}
			
			points = []
			for point in path:
				points.append( tuple( [float(x) * SCALE for x in point.split(',')]))
			batch[edge_count] = points
			last_frame = frame
		if len(batch) > 0:
			yield batch

def values_reader(input_file):
	with open(input_file, 'r') as handle:
		next_line = handle.readline()
		schema = []
		while next_line:
			line = next_line.strip().split("\t")
			parts = line[1:]
			next_line = handle.readline()
			if 'Frame' == line[0]:
				schema = [int(x) for x in parts]
				continue
			
			pairs = [(float(parts[i]), i) for i in range(len(parts))]
			ordered = sorted(pairs, reverse = True)
			best_pair = ordered[0]
			best_edge_count = schema[ best_pair[1] ]
			
			yield best_edge_count

def get_frames(skeleton_csv, pcl_csv, paths_csv, best_values):
	skeleton_reader = csv_reader(skeleton_csv)
	pcl_reader = csv_reader(pcl_csv)
	path_reader = paths_reader(paths_csv)
	value_reader = values_reader(best_values)
	
	
	skeleton_frame = skeleton_reader.next()
	pcl_frame = pcl_reader.next()
	path_frame = path_reader.next()
	value = value_reader.next()
	
	while skeleton_frame and pcl_frame and path_frame and value:
		frame = (skeleton_frame, pcl_frame, path_frame, value)
		yield frame
		skeleton_frame = skeleton_reader.next()
		pcl_frame = pcl_reader.next()
		path_frame = path_reader.next()
		value = value_reader.next()

FRAME_N = 0
DIM = 1536
def start_visualizing(cluster_points):
	window = app.Window(DIM,DIM, color=(1,2,1,1))
	point_collection = PointCollection("agg", color="local", size="local")
	paths = PathCollection(mode="agg")
	
	@window.event
	def on_draw(dt):
		global FRAME_N
		window.clear()
		point_collection.draw()

		time.sleep(DRAW_DELAY)
		if not cluster_points.empty():
			possible = cluster_points.get()
			
			print "frame",FRAME_N
			FRAME_N += 1
			
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
						new_hop =[round(0.7 * abs(x),5) for x in hop]
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
	

def process_files(skeleton_csv, pcl_csv, best_paths_csv, cluster_points, best_values):
	# edge_count_to_show = 4
	frame_n = 0
	for skeleton_frame, pcl_frame, path_frame, edge_count_to_show in get_frames(skeleton_csv, pcl_csv, best_paths_csv, best_values):
		frame_n += 1
		
		if EDGE_COUNT_OVERRIDE:
			edge_count_to_show = EDGE_COUNT_OVERRIDE
		
		to_render = pcl_frame
		
		if len(to_render) > MAX_POINTS:
			to_render = random.sample(to_render, MAX_POINTS)
		
		for point in to_render:
			cluster_points.put( (POINT, point, CLUSTER_COLOR, BIG) )

		for point in skeleton_frame:
			cluster_points.put( (POINT, point, SKELETON_COLOR, BIG) )
		
		this_path = path_frame[edge_count_to_show]
		
		for point in this_path:
			cluster_points.put( (POINT, point, LINE_COLOR, BIG) )
		
		
		endpoints = [(this_path[i], this_path[i+1]) for i in range(len(this_path)-1)]
		for line in [generate_line(p0,p1) for p0,p1 in endpoints]:
			for point in line:
				cluster_points.put( (POINT, point, LINE_COLOR, SMALL) )
		
		cluster_points.put(TERMINATOR)
	

if '__main__' == __name__:
	if len(sys.argv) < 3:
		print "python",sys.argv[0],"skeleton.csv pcl.csv best_paths.csv best_values.csv"
		exit()
	
	skeleton_csv, pcl_csv, best_paths_csv, best_values_csv = sys.argv[1:5]
	
	cluster_points = multiprocessing.Queue()
	
	# -0.572,-0.343,1.101	 -0.398,-0.562,1.183	 -0.364,-0.436,1.223	 -0.299,0.04,1.217
	# 26
	
	processor = multiprocessing.Process(target = process_files, args = (skeleton_csv, pcl_csv, best_paths_csv, cluster_points, best_values_csv))
	processor.start()
	
	start_visualizing(cluster_points)
	
	
	
	
	
	
