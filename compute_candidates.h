
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
	short distance;
}sort_pair;

typedef struct path{
	short num_points;
	point points[MAX_VERTICES];
	bool visited_points[MAX_POINTS_IN_SKELETON_FRAME];
	short num_visited;
} path;

typedef struct stateless_path{
	short num_points;
	point *points[MAX_VERTICES];
} stateless_path;

typedef struct candidate{
	short num_lengths;
	short lengths[MAX_EDGES];
	short total_length;
} candidate;



double v_dot(point *u, point *v){
	return (u->x * v->x) + (u->y * v->y) + (u->z * v->z);
}
double v_norm(point *v){
	return sqrt(v_dot(v,v));
}
void vector_between(point *u, point *v, point *vector){
	vector->x = u->x - v->x;
	vector->y = u->y - v->y;
	vector->z = u->z - v->z;
}
double v_dist(point *u, point *v){
	point vector;
	vector_between(u,v,&vector);
	return v_norm(&vector);
}

unsigned short distance_to_segment(point *p, point *s0, point *s1){
	
	point v;
	vector_between(s1, s0, &v);
	point w;
	vector_between(p, s0, &w);
	double c1 = v_dot(&w, &v);
	if(c1 < 0){
		return floorf(v_dist(p, s0));
	}
	double c2 = v_dot(&v, &v);
	if(c2 <= c1){
		return floorf(v_dist(p, s1));
	}
	double b = c1 / c2;
	
	v.x = s0->x + b * v.x;
	v.y = s0->y + b * v.y;
	v.z = s0->z + b * v.z;
	
	return floorf(v_dist(p, &v));
}



void validate_candidates(int rank, int num_candidates, candidate *candidates){
	int i,j;
	for(i = 0; i < num_candidates; i++){
		
		if(candidates[i].num_lengths == 0){
			printf("%i found invalid candidate\n", rank);
			exit(1);
		}
		for(j = 0; j < candidates[i].num_lengths; j++){
			if(candidates[i].lengths[j] <= 0){
				printf("\n\n%i Invalid length of <= 0\n", rank);
				exit(1);
			}
		}
	}
}


int sort_pairwise_distances(const void *a, const void *b){
	//	printf("COMPARING %i and %i\n",((sort_pair *)a)->distance,((sort_pair *)b)->distance);
	return ((sort_pair *)a)->distance > ((sort_pair *)b)->distance ? 1 : -1;
}

void get_pairwise_distances(int num_points, sort_pair **pairs, frame *frm){
	(*pairs) = (sort_pair *) malloc (num_points * num_points * sizeof(sort_pair));
	int i,j;
	int offset;
	point *p0, *p1;
	sort_pair *this_pair;
	for(i = 0; i < num_points; i++){
		p0 = &(frm->points[i]);
		for(j = 0; j < num_points; j++){
			offset = (i * num_points + j); // i = rows, j = cols
			
			p1 = &(frm->points[j]);
			(*pairs)[offset].pid = p1->pid;
			if(i == j){
				(*pairs)[offset].distance = 0;
			}else{
				(*pairs)[offset].distance = floorf(euclid_distance(p0, p1));
			}
//			printf("1) Dist bet (%i) %i,%i,%i and (%i) %i,%i,%i = %i\n", p0->pid, p0->x, p0->y, p0->z, p1->pid, p1->x, p1->y, p1->z, (*pairs)[offset].distance);
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
	if(((point *) a)->y == ((point *) b)->y){
		return ((point *) a)->x > ((point *) b)->x ? 1 : -1;
	}
	return ((point *) a)->y > ((point *) b)->y ? 1 : -1;
}

void get_anchors(int *num_anchors, point ***anchors, frame *frm){
	// assumes frm points are already sorted by y value
	short anchor_threshold = 40; // 4 centimeters
	short bottom_most = 0;
	int i;
	for(i = 0; i < frm->num_points; i++){
		if(i == 0){
			bottom_most = frm->points[i].y;
			(*num_anchors)++;
			continue;
		}
		
		if(abs(frm->points[i].y - bottom_most) <= anchor_threshold){
			(*num_anchors)++;
		}
	}
	(*anchors) = (point **) malloc ((*num_anchors) * sizeof(point *));
	for(i = 0; i < *num_anchors; i++){
		(*anchors)[i] = &(frm->points[i]);
//		printf("point %i: (pid: %i) %i,%i,%i\n", i, (*anchors)[i]->pid, (*anchors)[i]->x, (*anchors)[i]->y, (*anchors)[i]->z);
	}
}

void print_path(path *path){
	printf("Path: (%i long)\n", path->num_points);
	int i;
	for(i = 0; i < path->num_points; i++){
		printf("\t(%i)\t%i\t%i\t%i\n", path->points[i].pid, path->points[i].x, path->points[i].y, path->points[i].z);
	}
}

void print_candidate(candidate *cand){
	printf("Candidate: (%i long)\n", cand->num_lengths);
	int i;
	for(i = 0; i < cand->num_lengths; i++){
		printf("\t%i", cand->lengths[i]);
	}
	printf("\n");
}

short get_num_closest(num_points){
	return MIN(BRANCH_NEIGHBORS, num_points);
}

bool points_are_equal(point *p0, point *p1){
	return p0->pid == p1->pid; // breaks if two points are exactly the same, highly unlikely
	//	return p0->x == p1->x && p0->y == p1->y && p0->z == p1->z;
}

bool in_path(path *path, point *point){
	int i;
	for(i = 0; i < path->num_points; i++){
		if(points_are_equal(point, &(path->points[i]))){
			return true;
		}
	}
	return false;
}


path* initialize_stack_with_anchors(frame *frame, int *stack_size, int *space_on_stack){
	int num_points = frame->num_points;
	
	int num_anchors = 0;
	point **anchors = NULL;
	get_anchors(&num_anchors, &anchors, frame);
	path *stack = NULL;
	int i,j;
	for(i = 0; i < num_anchors; i++){
		stack = get_more_space_and_copy(space_on_stack, stack, *stack_size, PATHS, sizeof(path));
		
		memcpy(&(stack[*stack_size].points[0] ), anchors[i], sizeof(point));
		stack[*stack_size].num_points = 1;
		
		// don't let anchors connect to anchors
		for(j = 0; j < num_anchors; j++){
			stack[*stack_size].visited_points[anchors[j]->pid] = true;
			stack[*stack_size].num_visited++;
		}
		(*stack_size)++;
	}
	if(anchors){
		free(anchors);
	}
	return stack;
}


stateless_path* initialize_stack_with_anchors_stateless(frame *frame, int *stack_size, int *space_on_stack){
	int num_points = frame->num_points;
	
	int num_anchors = 0;
	point **anchors = NULL;
	get_anchors(&num_anchors, &anchors, frame);
	stateless_path *stack = NULL;
	int i;
	for(i = 0; i < num_anchors; i++){
		stack = get_more_space_and_copy(space_on_stack, stack, *stack_size, PATHS, sizeof(stateless_path));
		
		stack[*stack_size].points[0] = anchors[i];
		stack[*stack_size].num_points = 1;
		(*stack_size)++;
	}
	if(anchors){
		free(anchors);
	}
	return stack;
}

bool are_in_line(point *p, point *p0, point *p1){
	unsigned short threshold_distance = 30; // millimeters off the line
	unsigned short distance = distance_to_segment(p, p0, p1);
	
//	printf("Checking: \n");
//	printf("\t%i,%i,%i\n", p->x, p->y, p->z);
//	printf("\tis %i from:\n", distance);
//	printf("\t%i,%i,%i\n", p0->x, p0->y, p0->z);
//	printf("\t%i,%i,%i\n", p1->x, p1->y, p1->z);
	
	return distance <= threshold_distance;
}

void convert_path_to_candidate(int rank, candidate *cand, path *path){
	// turn paths to length edges
	int j,k;
	double value;
	memset(cand, 0, sizeof(candidate));
	for(k = 0; k < path->num_points - 1; k++){
		
		value = euclid_distance(&(path->points[k]),&(path->points[k+1]));
		cand->lengths[k] = floorf(value);
		
		if(cand->lengths[k] <= 0){
			printf("%i Just added a length <= 0 candidate\n", rank);
			printf("%i, %i, %i\n",path->points[k].x, path->points[k].y, path->points[k].z);
			printf("%i, %i, %i\n",path->points[k+1].x, path->points[k+1].y, path->points[k+1].z);
			exit(1);
		}
		//				printf("Added length: %f between %f,%f,%f and %f,%f,%f\n", candidates[j].lengths[k], paths[j].points[k].x,paths[j].points[k].y,paths[j].points[k].z, paths[j].points[k+1].x,paths[j].points[k+1].y,paths[j].points[k+1].z);
		
		cand->num_lengths++;
	}
	
}

bool is_same_candidate(candidate *cand_1, candidate *cand_2){
	if(cand_1->num_lengths != cand_2->num_lengths){
		return false;
	}
	int i;
	for(i = 0; i < cand_1->num_lengths; i++){
		if(cand_1->lengths[i] != cand_2->lengths[i]){
			return false;
		}
	}
	return true;
}

void deduplicate_candidates(int rank, int *num_candidates, candidate *candidates, int last_unique_index, bool verbose){
	int i,j;
	int num_duplicates = 0;
	int was_count = *num_candidates;
	for(i = 0; i < ((*num_candidates)-1); i++){
		for(j = i+last_unique_index+1; j < (*num_candidates); ){
			
			if(is_same_candidate(&(candidates[i]), &(candidates[j]))){
				num_duplicates++;
				if(j < (*num_candidates)-1){
					// at least one left

//					memmove(&(candidates[j]), &(candidates[j+1]), ((*num_candidates) - (j + 1)) * sizeof(candidate));
					memcpy(&(candidates[j]), &(candidates[*num_candidates-1]), sizeof(candidate));
				}
				(*num_candidates)--;
				
				memset(&(candidates[*num_candidates]), 0 ,sizeof(candidate));
				
			}else{
				// only move j if we didn't shift any above, since that would have moved something else into the jth spot
				j++;
			}
		}
	}
	if(verbose){
		printf("\t%i Found %i duplicates, from %i to %i\n", rank, num_duplicates, was_count, *num_candidates);
	}
}

void compute_candidates_for_frame(int rank, int frame_n, frame *frm, int *num_candidates, candidate **candidates){
	//	printf("> %i cand(f_%i)\n", rank, frame_n);
	int i,j;
//	printf("Frame points %i\n", frm->num_points);
	for(j = 0; j < frm->num_points; j++){
		if(frm->points[j].x == 0 || frm->points[j].y == 0 || frm->points[j].z == 0){
			for(i = 0; i < frm->num_points; i++){
				printf("%i %i %i\n", frm->points[i].x, frm->points[i].y,frm->points[i].z);
			}
			exit(1);
		}
	}
	
	
	int num_points = frm->num_points;
	
	// sort points by their y value
	qsort(frm->points, frm->num_points, sizeof(point), sort_by_y_value);
	
	for(i = 0; i < num_points; i++){
		frm->points[i].pid = i;
	}
	
	sort_pair *pairs;
	get_pairwise_distances(num_points, &pairs, frm);
	//	printf("inside compute cands\n");
	
	int space_for_candidates = 0;
	(*candidates) = get_more_space_and_copy(&space_for_candidates, (*candidates), (*num_candidates), PATHS, sizeof(candidate));
	
	int space_on_stack = 0;
	int stack_size = 0;
	path *stack = initialize_stack_with_anchors(frm, &stack_size, &space_on_stack);
	
	short num_closest = get_num_closest(num_points);
	sort_pair *nearest_pair;
	point *nearest_point;
	path current_path;
	point *last_point;
	
	path *path_to_analyze;
	point inline_points[3];
	point *p0,*p1,*p2;
	
	bool changed;
	int offset;
	
	int new_candidates = 0;
	unsigned int last_unique_index = 0;
	
	while(stack_size > 0){
		// get the last item
		memset(&current_path, 0, sizeof(path));
		memcpy(&current_path, &(stack[stack_size-1]), sizeof(path));
		//		printf("current: %f,%f,%f of %i\n", current_path.points[0].x,current_path.points[0].y,current_path.points[0].z,current_path.num_points);
		
		// decrementing this means we're working on the back of the stack.
		stack_size--;
		
//		printf("\n\nCurrent ");
//		print_path(&current_path);
//		printf("already visited: %i\n", current_path.num_visited);
//		printf("%i length: %i\n", rank, current_path.num_points);
		for(i = 0; i < current_path.num_points; i++){
			if(current_path.points[i].x == 0 || current_path.points[i].y == 0 || current_path.points[i].z == 0){
				printf("%i found invalid path point, zero:\n", rank);
				print_path(&current_path);
				
				printf("%i last point: %i %i %i\n", rank, nearest_point->x, nearest_point->y, nearest_point->z);
				exit(1);
			}
		}
		
		if(current_path.num_points == MAX_VERTICES || num_points == current_path.num_visited){
			// only done once we've visited all points. Path is considered minimized since we minimized during creation
			
//			printf("adding candidate: %i %i\n", space_for_candidates, *num_candidates);
			(*candidates) = get_more_space_and_copy(&space_for_candidates, (*candidates), (*num_candidates), PATHS, sizeof(candidate));
			
			convert_path_to_candidate(rank, &((*candidates)[*num_candidates]), &current_path);
			
//			printf("(now %i paths finished), latest ", (*num_paths)+1);
//			print_path(&((*paths)[*num_paths]));
			
			(*num_candidates)++;
			new_candidates++;
			
			if((*num_candidates) == space_for_candidates){
//				printf("%i Trying dedupe %i from %i %i\n", rank, *num_candidates, last_unique_index, num_points);
				
				deduplicate_candidates(rank, &new_candidates, &((*candidates)[last_unique_index]), 0, false);
//				printf("%i mid to %i\n", rank, new_candidates);
				(*num_candidates) = last_unique_index + new_candidates;
				
				deduplicate_candidates(rank, num_candidates, *candidates, last_unique_index, false);
//				printf("%i down to %i\n", rank, *num_candidates);
				last_unique_index = *num_candidates;
				new_candidates = 0;
			}
		
			continue;
		}
		
		
		last_point = &(current_path.points[ current_path.num_points - 1]);
		
		for(i = 1; i < num_points; i++){
			
			/// start at 1 because 0th 'nearest' is itself with 0 distance
			offset = (last_point->pid * num_points + i); // pid = rows, i = cols
			nearest_pair = &(pairs[offset]);
			nearest_point = &(frm->points[nearest_pair->pid]);
			//			printf("\tnearest to last_point (%i): (%i) %f,%f,%f at dist %f\n", last_point->pid, nearest_point->pid, nearest_point->x, nearest_point->y, nearest_point->z, nearest_pair->distance);
			
//			printf("trying %i    %i %i %i\n", nearest_point->pid, nearest_point->x, nearest_point->y, nearest_point->z);
			
//			if(in_path(&current_path, nearest_point)){
			if(current_path.visited_points[nearest_point->pid]){
				// point has already been visited
//				printf("\t>>> Skipping point (%i), already visited\n", nearest_point->pid);
				continue;
			}
			
			
			if(nearest_point->x == 0 || nearest_point->y == 0 || nearest_point->z == 0){
				printf("offset %i, pair %i %i\n", offset, nearest_pair->pid, nearest_pair->distance);
				printf("%i %i %i %i\n", frm->num_points, frm->points[nearest_pair->pid].x, frm->points[nearest_pair->pid].y, frm->points[nearest_pair->pid].z);
				printf("nearest_point is zero\n");
				exit(1);
			}
			
			stack = get_more_space_and_copy(&space_on_stack, stack, stack_size, PATHS, sizeof(path));
			//			printf("\tINNER stack now has space for %i, has %i\n", space_on_stack, stack_size);
			
			// copy current path
			memcpy(&(stack[stack_size]), &current_path, sizeof(path));
			// append the nearest point into the path
			// doing this order avoids extra memcpys into/from a temp
			memcpy(&(stack[stack_size].points[current_path.num_points]), nearest_point, sizeof(point));
			stack[stack_size].num_points++;
			
			stack[stack_size].visited_points[nearest_point->pid] = true;
			stack[stack_size].num_visited++;
			//			printf("\tJust pushed on ");
			//			print_path(&(stack[stack_size]));
			
			path_to_analyze = &(stack[stack_size]);
			// do the point-dropping, in case
//			printf("\n\nCurrent ");
//			print_path(path_to_analyze);
			
			changed = true;
			while(changed){
				changed = false;
				
				for(j = 0; j < (path_to_analyze->num_points-2); j++){
//					printf("%i, %i\n", j, (path_to_analyze->num_points-2));
//					fflush(stdout);
					memcpy(inline_points, &(stack[stack_size].points[j]), 3 * sizeof(point));
					
					qsort(inline_points, 3, sizeof(point), sort_by_y_value);
					
					
					// p1 is between (p0,p2)
					p0 = &(inline_points[0]);
					p1 = &(inline_points[1]);
					p2 = &(inline_points[2]);
					
//					printf("POINTS: %i %i %i\n", p0->pid, p1->pid, p2->pid);
					
					if(are_in_line(p1, p0, p2)){
						changed = true;
//						printf("IN LINE!\n");
						
						// p1 is between p0/p2, so remove it by copying 0/2 into spots and shifting later ones down and decrementing
						memcpy(&(path_to_analyze->points[j]), &(inline_points[0]), sizeof(point));
						memcpy(&(path_to_analyze->points[j+1]), &(inline_points[2]), sizeof(point));
						
//						printf("has %i, moving %i\n", path_to_analyze->num_points, path_to_analyze->num_points - (j+3));
						memmove(&(path_to_analyze->points[j+2]), &(path_to_analyze->points[j+3]), path_to_analyze->num_points - (j+3));
						
						path_to_analyze->num_points--;
						
						
//						printf("\n\nNew ");
//						print_path(path_to_analyze);
					}
					
//					printf("\n\n");
				}
				
			}
//			printf("\n\nFinal ");
//			print_path(path_to_analyze);
//			printf("done\n");
			
			stack_size++;
			
		}
		
		//		exit(1);
	}
	
	
	free(stack);
	free(pairs);
}


void compute_candidate_for_frames(int rank, int num_skeleton_frames, int my_start, frame *skeleton_frames){
	printf("> %i About to compute candidates for %i frames\n", rank, num_skeleton_frames);
	//
	//	point p0 = {1,1,1};
	//	point p1 = {1,2,2};
	//	double dist = euclid_distance(&p0, &p1);
	//	printf("distance: %f\n", dist);
	//
	
	candidate *candidates = NULL;
	int num_candidates = 0;
	
	int i;
	int frame_n;
	for(i = 0; i < num_skeleton_frames; i++){
		frame_n = my_start + i;
		
		compute_candidates_for_frame(rank, frame_n, &(skeleton_frames[i]), &num_candidates, &candidates);
		
		break;
	}
	printf("%i Final dedupe\n", rank);
	deduplicate_candidates(rank, &num_candidates, candidates, 0, true);
	
	printf("Rank %i has %i candidates\n", rank, num_candidates);
	fflush(stdout);
	
	validate_candidates(rank, num_candidates, candidates);
	
	printf("Rank %i sending\n", rank);
	fflush(stdout);
	
	// gather the number of candidates
	MPI_Gather(&num_candidates, 1, MPI_INT, NULL, 1, MPI_INT, 0, MPI_COMM_WORLD);
	
	MPI_Gatherv(candidates, num_candidates * sizeof(candidate), MPI_BYTE, NULL, 0, NULL, MPI_BYTE, 0, MPI_COMM_WORLD);
	
	if(candidates){
		free(candidates);
	}
}

