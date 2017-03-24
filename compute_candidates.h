
#define MAX(x, y) ((x) > (y) ? (x) : (y))
#define MIN(x, y) ((x) < (y) ? (x) : (y))

double euclid_distance(point *p0, point *p1){
	double diff_x = p0->x - p1->x;
	double diff_y = p0->y - p1->y;
	double diff_z = p0->z - p1->z;
	
	return sqrt( (diff_x * diff_x) + (diff_y * diff_y) + (diff_z * diff_z));
}

typedef struct sort_pair{
	int pid; // pid referencing which point the distance is to
	double distance;
}sort_pair;

typedef struct path{
	short num_points;
	point points[MAX_VERTICES];
} path;

typedef struct candidate{
	short num_lengths;
	double lengths[MAX_EDGES];
} candidate;

int sort_pairwise_distances(const void *a, const void *b){
	//	printf("COMPARING %f and %f\n",((sort_pair *)a)->distance,((sort_pair *)b)->distance);
	return ((sort_pair *)a)->distance > ((sort_pair *)b)->distance ? 1 : -1;
}

void get_pairwise_distances(int rank, int num_points, sort_pair **pairs, frame *frm){
	(*pairs) = (sort_pair *) malloc (num_points * num_points * sizeof(sort_pair));
	
	int offset;
	point *p0, *p1;
	sort_pair *this_pair;
	for(int i = 0; i < num_points; i++){
		p0 = &(frm->points[i]);
		for(int j = 0; j < num_points; j++){
			offset = (i * num_points + j); // i = rows, j = cols
			
			p1 = &(frm->points[j]);
			(*pairs)[offset].pid = p1->pid;
			if(i == j){
				(*pairs)[offset].distance = 0.0;
			}else{
				(*pairs)[offset].distance = euclid_distance(p0, p1);
			}
			//			printf("1) Dist bet (%i) %f,%f,%f and (%i) %f,%f,%f = %f\n", p0->pid, p0->x, p0->y, p0->z, p1->pid, p1->x, p1->y, p1->z, (*pairs)[offset].distance);
		}
		
		// got distances, now sort them inside each row
		
		qsort(&((*pairs)[i * num_points]), num_points, sizeof(sort_pair), sort_pairwise_distances);
		
		/*
		 // to check sort worked
		 for(int j = 0; j < num_points; j++){
			offset = (i * num_points + j); // i = rows, j = cols
			this_pair = &((*pairs)[offset]);
			
			p1 = &(frm->points[ this_pair->pid ]);
			
			printf("2) Dist bet (%i) %f,%f,%f and (%i) %f,%f,%f = %f\n", p0->pid, p0->x, p0->y, p0->z, p1->pid, p1->x, p1->y, p1->z, (*pairs)[offset].distance);
		 }
		 printf("\n\n");
		 */
	}
}

int sort_by_y_value(const void *a, const void *b){
	return ((point *) a)->y > ((point *) b)->y ? 1 : -1;
}

void get_anchors(int num_anchors, point **anchors, frame *frm){
	for(int i = 0; i < num_anchors; i++){
		anchors[i] = &(frm->points[i]);
		//		printf("point %i: (pid: %i) %f,%f,%f\n", i, anchors[i]->pid, anchors[i]->x, anchors[i]->y, anchors[i]->z);
	}
}

void print_path(path *path){
	printf("Path: (%i long)\n", path->num_points);
	for(int i = 0; i < path->num_points; i++){
		printf("\t(%i)\t%f\t%f\t%f\n", path->points[i].pid, path->points[i].x, path->points[i].y, path->points[i].z);
	}
}

short get_num_closest(num_points){
	return MIN(BRANCH_NEIGHBORS, num_points);
}

bool points_are_equal(point *p0, point *p1){
	return p0->pid == p1->pid; // breaks if two points are exactly the same, highly unlikely
	//	return p0->x == p1->x && p0->y == p1->y && p0->z == p1->z;
}

bool in_path(path *path, point *point){
	for(int i = 0; i < path->num_points; i++){
		if(points_are_equal(point, &(path->points[i]))){
			return true;
		}
	}
	return false;
}

void compute_candidates_for_frame(int rank, int frame_n, int num_vertices, frame *frm, int *num_paths, path **paths){
	//	printf("> %i cand(f_%i) %i vertices\n", rank, frame_n, num_vertices);
	
	int num_points = frm->num_points;
	
	// sort points by their y value
	qsort(frm->points, frm->num_points, sizeof(point), sort_by_y_value);
	
	for(int i = 0; i < num_points; i++){
		frm->points[i].pid = i;
	}
	
	int num_anchors = MIN(MAX_ANCHORS, num_points);
	point *anchors[num_anchors];
	get_anchors(num_anchors, anchors, frm);
	
	
	sort_pair *pairs;
	get_pairwise_distances(rank, num_points, &pairs, frm);
	//	printf("inside compute cands\n");
	
	int space_for_paths = 0;
	(*paths) = get_more_space_and_copy(&space_for_paths, (*paths), (*num_paths), PATHS, sizeof(path));
	
	int space_on_stack = 0;
	int stack_size = 0;
	path *stack = NULL;
	
	for(int i = 0; i < num_anchors; i++){
		stack = get_more_space_and_copy(&space_on_stack, stack, stack_size, PATHS, sizeof(path));
		//		printf("OUTER stack now has space for %i, has %i\n", space_on_stack, stack_size);
		
		memcpy(&(stack[stack_size].points[0] ), anchors[i], sizeof(point));
		stack[stack_size].num_points = 1;
		//		print_path(&(stack[stack_size]));
		stack_size++;
	}
	
	short num_closest = get_num_closest(num_points);
	sort_pair *nearest_pair;
	point *nearest_point;
	path current_path;
	point *last_point;
	
	int offset, added;
	
	while(stack_size > 0){
		// get the last item
		memcpy(&current_path, &(stack[stack_size-1]), sizeof(path));
		//		printf("current: %f,%f,%f of %i\n", current_path.points[0].x,current_path.points[0].y,current_path.points[0].z,current_path.num_points);
		
		// decrementing this means we're working on the back of the stack.
		stack_size--;
		
		//		printf("\n\nCurrent ");
		//		print_path(&current_path);
		
		if(num_vertices == current_path.num_points){
			(*paths) = get_more_space_and_copy(&space_for_paths, (*paths), (*num_paths), PATHS, sizeof(path));
			memcpy( &((*paths)[*num_paths]), &current_path, sizeof(path));
			//			printf("(now %i paths finished), latest ", (*num_paths)+1);
			//			print_path(&((*paths)[*num_paths]));
			(*num_paths)++;
			continue;
		}
		
		
		last_point = &(current_path.points[ current_path.num_points - 1]);
		added = 0;
		// either stop after added have been added or try all the points
		for(int i = 1; i < num_points && added < num_closest; i++){
			
			/// start at 1 because 0th 'nearest' is itself with 0 distance
			offset = (last_point->pid * num_points + i); // pid = rows, i = cols
			nearest_pair = &(pairs[offset]);
			nearest_point = &(frm->points[nearest_pair->pid]);
			//			printf("\tnearest to last_point (%i): (%i) %f,%f,%f at dist %f\n", last_point->pid, nearest_point->pid, nearest_point->x, nearest_point->y, nearest_point->z, nearest_pair->distance);
			
			if(in_path(&current_path, nearest_point)){
				// point is already in path, skip it
				//				printf("\t>>> Skipping point (%i), already in path\n", nearest_point->pid);
				continue;
			}
			
			
			stack = get_more_space_and_copy(&space_on_stack, stack, stack_size, PATHS, sizeof(path));
			//			printf("\tINNER stack now has space for %i, has %i\n", space_on_stack, stack_size);
			
			// copy current path
			memcpy(&(stack[stack_size]), &current_path, sizeof(path));
			// append the nearest point into the path
			// doing this order avoids extra memcpys into/from a temp
			memcpy(&(stack[stack_size].points[current_path.num_points]), nearest_point, sizeof(point));
			stack[stack_size].num_points++;
			
			//			printf("\tJust pushed on ");
			//			print_path(&(stack[stack_size]));
			
			stack_size++;
			added++;
			
		}
		
		//		exit(1);
	}
	
	
	free(stack);
	free(pairs);
}

bool is_same_path(path *path_1, path *path_2){
	if(path_1->num_points != path_2->num_points){
		return false;
	}
	
	for(int i = 0; i < path_1->num_points; i++){
		if(!points_are_equal(&(path_1->points[i]), &(path_2->points[i]))){
			return false;
		}
	}
	return true;
}

void deduplicate_paths(path *paths, int num_paths){
	for(int i = 0; i < num_paths; i++){
		for(int j = i+1; j < num_paths; ){
			
			if(is_same_path(&(paths[i]), &(paths[j]))){
				printf("found duplicate\n");
				if(j < num_paths-1){
					// shift the remaining paths down one
					memmove(&(paths[j]), &(paths[j+1]), num_paths - j - 1);
				}
				num_paths--;
				
			}else{
				// only move j if we didn't memmove, since that would have moved something else into the jth spot
				j++;
			}
		}
	}
	
}

void compute_candidate_for_frames(int rank, int num_skeleton_frames, int my_start, frame *skeleton_frames){
	printf("> %i About to compute candidates for %i frames\n", rank, num_skeleton_frames);
	//
	//	point p0 = {1,1,1};
	//	point p1 = {1,2,2};
	//	double dist = euclid_distance(&p0, &p1);
	//	printf("distance: %f\n", dist);
	//
	path *paths = NULL;
	int num_paths = 0;
	
	int frame_n;
	for(int i = 0; i < num_skeleton_frames; i++){
		frame_n = my_start + i;
		
		int batch_start = 0;
		for(int num_vertices = MIN_VERTICES; num_vertices <= MAX_VERTICES; num_vertices++){
			batch_start = num_paths;
			
			compute_candidates_for_frame(rank, frame_n, num_vertices, &(skeleton_frames[i]), &num_paths, &paths);
			//			printf(">> was %i cand(f_%i) %i vertices => now %i paths\n", rank, frame_n, num_vertices, num_paths);
			
			// de-dupe paths, exceedingly unlikely, since there won't be duplicated skeleton points.
			// if there is a duplication, something went wrong in path generation above.
			//			deduplicate_paths(&(paths[batch_start]), num_paths - batch_start);
			
			//			printf(">> %i cand(f_%i) %i vertices => now %i paths\n", rank, frame_n, num_vertices, num_paths);
		}
		
		
		
		//		break;
	}
	
	// turn paths to length edges
	candidate *candidates = (candidate *) malloc (num_paths * sizeof(candidate));
	memset(candidates, 0, num_paths * sizeof(candidate));
	
	for(int j = 0; j < num_paths; j++){
		for(int k = 0; k < paths[j].num_points - 1; k++){
			candidates[j].lengths[k] = euclid_distance(&(paths[j].points[k]),&(paths[j].points[k+1]));
			
			//				printf("Added length: %f between %f,%f,%f and %f,%f,%f\n", candidates[j].lengths[k], paths[j].points[k].x,paths[j].points[k].y,paths[j].points[k].z, paths[j].points[k+1].x,paths[j].points[k+1].y,paths[j].points[k+1].z);
			
			candidates[j].num_lengths++;
		}
		//			printf("\n\n");
	}
	
	//	printf("> %i %i lengths\n", rank, num_paths);
	
	// gather the number of candidates
	MPI_Gather(&num_paths, 1, MPI_INT, NULL, 1, MPI_INT, 0, MPI_COMM_WORLD);
	
	MPI_Gatherv(candidates, num_paths * sizeof(candidate), MPI_BYTE, NULL, 0, NULL, MPI_BYTE, 0, MPI_COMM_WORLD);
	
	if(paths){
		free(paths);
	}
}
