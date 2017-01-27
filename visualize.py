import sys, json, multiprocessing, time, random
import numpy as np

TERMINATOR = "terminate"
DRAW_DELAY = 1.0
MAX_POINTS = 1000

PURPLE = [ 0.44865016 , 0.08414224 , 0.81931686 , 0.95626713]
BLUE = [ 0.20815755, 0.4907831, 0.72991901, 0.31302678]
PINK = [ 0.8650541  , 0.19603308 , 0.67572193 , 0.44581312]
ORANGE = [ 0.83885254 , 0.48906794 , 0.02429835 , 0.30316865]
LURPLE = [ 0.81915068 , 0.56576916 , 0.98879524 , 0.0699857 ]
RED = [ 0.9135442  , 0.48970524 , 0.56584265 , 0.68154473]

CLUSTER = RED
SKELETON = BLUE


from glumpy import app
from glumpy.graphics.collections import PointCollection

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
            # print path
            points = []
            for point in path:
                points.append( tuple( [float(x) for x in point.split(',')]))
            batch[edge_count] = points
            last_frame = frame
        if len(batch) > 0:
            yield batch

def get_frames(skeleton_csv, pcl_csv, paths_csv):
    skeleton_reader = csv_reader(skeleton_csv)
    pcl_reader = csv_reader(pcl_csv)
    path_reader = paths_reader(paths_csv)
    
    
    skeleton_frame = skeleton_reader.next()
    pcl_frame = pcl_reader.next()
    path_frame = path_reader.next()
    
    while skeleton_frame and pcl_frame and path_frame:
        yield (skeleton_frame, pcl_frame, path_frame)
        skeleton_frame = skeleton_reader.next()
        pcl_frame = pcl_reader.next()
        path_frame = path_reader.next()


def start_visualizing(cluster_points):
    window = app.Window(1024,1024, color=(1,1,1,1))
    point_collection = PointCollection("agg", color="local", size="local")

    @window.event
    def on_draw(dt):
        window.clear()
        point_collection.draw()

        time.sleep(DRAW_DELAY)
        if not cluster_points.empty():
            possible = cluster_points.get()
            
            while len(point_collection) > 0:
                del point_collection[0]
            
            while not possible == TERMINATOR:
                point, color = possible
                new_point = [[0.7*x for x in point]]
                new_point[0][1] *= -1
                point_collection.append(new_point,
                                  color = color,
                                  size  = 10)
                possible = cluster_points.get()

    window.attach(point_collection["transform"])
    window.attach(point_collection["viewport"])
    app.run()

def process_files(skeleton_csv, pcl_csv, best_paths_csv, cluster_points):
    edge_count_to_show = 3
    frame = 0
    for skeleton_frame, pcl_frame, path_frame in get_frames(skeleton_csv, pcl_csv, best_paths_csv):
        frame += 1
        
        to_render = pcl_frame
        
        if len(to_render) > MAX_POINTS:
            to_render = random.sample(to_render, MAX_POINTS)
        
        for point in to_render:
            cluster_points.put( (point, CLUSTER) )
        
        for point in skeleton_frame:
            cluster_points.put( (point, SKELETON) )
        
        cluster_points.put(TERMINATOR)
    

if '__main__' == __name__:
    if len(sys.argv) < 3:
        print "python",sys.argv[0],"skeleton.csv pcl.csv best_paths.csv"
        exit()
    
    skeleton_csv, pcl_csv, best_paths_csv = sys.argv[1:4]
    
    cluster_points = multiprocessing.Queue()
    
    processor = multiprocessing.Process(target = process_files, args = (skeleton_csv, pcl_csv, best_paths_csv, cluster_points))
    processor.start()
    
    start_visualizing(cluster_points)
    
    
    
    
    
    
