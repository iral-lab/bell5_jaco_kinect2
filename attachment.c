#include <mpi.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>


#define MIN_PROCESSORS 4

#define MAX_EDGES 6
#define MAX_VERTICES MAX_EDGES + 1
#define MIN_VERTICES 3 // at least 2 edges

#define MAX_ANCHORS 3
#define BRANCH_NEIGHBORS 3

#define LAMBDA_SCALAR 1.1

#define SAMPLE_PCL_POINTS 1
// rate / 100 ~= sample%
#define PCL_POINTS_SAMPLE_RATE 30

#define FRAME_DELIMITER '='

#define EXPECTED_ARG_COUNT 3

#include "util.h"
#include "compute_candidates.h"

typedef struct scored_path{
	path path;
	double score;
}scored_path;

typedef struct score{
	candidate candidate;
	double *scores; // will store each frame's score in order
}score;

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

double distance_to_segment(point *p, point *s0, point *s1){
	
	point v;
	vector_between(s1, s0, &v);
	point w;
	vector_between(p, s0, &w);
	double c1 = v_dot(&w, &v);
	if(c1 < 0){
		return v_dist(p, s0);
	}
	double c2 = v_dot(&v, &v);
	if(c2 <= c1){
		return v_dist(p, s1);
	}
	double b = c1 / c2;
	
	v.x = s0->x + b * v.x;
	v.y = s0->y + b * v.y;
	v.z = s0->z + b * v.z;
	
	return v_dist(p, &v);
}

double get_error_to_path(path *path, frame *pcl_frame){
	double error = 0.0;
	
	point *p,*p0,*p1;
	double best_distance, this_distance;
	
	for(int i = 0; i < pcl_frame->num_points; i++){
		p = &(pcl_frame->points[i]);
		best_distance = 99999;
		
		for(int j = 0; j < path->num_points - 1; j++){
			p0 = &(path->points[j]);
			p1 = &(path->points[j+1]);
			
			this_distance = distance_to_segment(p, p0, p1);
			if(this_distance < best_distance){
				best_distance = this_distance;
			}
		}
		
		error += best_distance;
	}
	return error;
}

double score_path(path *path, frame *pcl_frame){
	double error = get_error_to_path(path, pcl_frame);
	// want the error to be a penalty, more error => lower score
	error *= -100;
	
	int edge_count = path->num_points - 1;
	double edge_penalty = expf(LAMBDA_SCALAR * edge_count);
	double total_penalty = error - edge_penalty;
	return total_penalty;
}

void score_candidates_against_frame(score *score, int frame_i, frame *pcl_frame, frame *skeleton_frame){
	candidate *candidate = &(score->candidate);
	
	int num_points = skeleton_frame->num_points;
	
	// sort skeleton points by their y value
	qsort(skeleton_frame->points, skeleton_frame->num_points, sizeof(point), sort_by_y_value);
	
	for(int i = 0; i < num_points; i++){
		skeleton_frame->points[i].pid = i;
	}
	
	sort_pair *pairs;
	get_pairwise_distances(num_points, &pairs, skeleton_frame);
	
	int max_path_vertices = candidate->num_lengths + 1;
	short num_closest = get_num_closest(num_points);
	
	int space_for_paths = 0;
	int num_paths = 0;
	scored_path *paths = get_more_space_and_copy(&space_for_paths, NULL, num_paths, PATHS, sizeof(scored_path));
	
	
	int space_on_stack = 0;
	int stack_size = 0;
	path *stack = initialize_stack_with_anchors(skeleton_frame, &stack_size, &space_on_stack);
	path current_path;
	point *last_point;
	point *other_point;
	point new_point;
	sort_pair *other_pair;
	
	int offset, added;
	double desired_length;
	
	int added_pids[num_closest];
	
	double best_score = -999999999;
	double temp_score;
	
	while(stack_size > 0){
		memcpy(&current_path, &(stack[stack_size-1]), sizeof(path));
		stack_size--;
		
		if(current_path.num_points == max_path_vertices){
//			paths = get_more_space_and_copy(&space_for_paths, paths, num_paths, PATHS, sizeof(scored_path));
//			memcpy( &(paths[num_paths]), &current_path, sizeof(path));
			
//			paths[num_paths].score = score_path(&current_path, pcl_frame);
			
			temp_score = score_path(&current_path, pcl_frame);
			best_score = MAX(best_score, temp_score);
			
//			printf("FINALIZED PATH with score of %f\n", paths[num_paths].score);
//			print_path(&(paths[num_paths].path));
//			printf("\n\n");
			
			num_paths++;
			continue;
		}
//		print_path(&current_path);
		
		last_point = &(current_path.points[ current_path.num_points - 1]);
		added = 0;
		desired_length = candidate->lengths[current_path.num_points-1];
		for(int i = 0; i < num_closest && added < num_points; i++){
			
//			printf("looking for a point %f away from %f,%f,%f\n", desired_length, last_point->x, last_point->y, last_point->z);
			
			// adds the point that is the closest to the current length away from the last point
			
			
			point *best_point = NULL;
			double smallest_error = 9999;
			double best_points_distance = 0;
			double error;
			bool already_added;
			for(int j = 1; j < num_points; j++){
				// start at 1 since 0 is itself
				offset = (last_point->pid * num_points + j);
				other_pair = &(pairs[offset]);
				already_added = false;
				for(int k = 0; k < added; k++){
					if(other_pair->pid == added_pids[k]){
						already_added = true;
						break;
					}
				}
				if(already_added){
					continue;
				}
				
				error = fabs(other_pair->distance - desired_length);
				if(error < smallest_error){
					other_point = &(skeleton_frame->points[other_pair->pid]);
					smallest_error = error;
					best_point = other_point;
					best_points_distance = other_pair->distance;
				}
			}
			
			if(!best_point){
				printf("SOMETHING WENT WRONG, no point was selected\n");
				exit(1);
			}
			
//			printf("adding pid %i at %f,%f,%f, which is %f away, error %f\n", best_point->pid, best_point->x, best_point->y, best_point->z, best_points_distance, smallest_error);
			
			// standard ending
			stack = get_more_space_and_copy(&space_on_stack, stack, stack_size, PATHS, sizeof(path));
			
			memcpy(&(stack[stack_size]), &current_path, sizeof(path));
			
			memcpy(&(stack[stack_size].points[current_path.num_points]), best_point, sizeof(point));
			stack[stack_size].num_points++;

			stack_size++;
			
			added_pids[added] = best_point->pid;
			added++;
			
		}
		
		
	}
	
	// now find the best score among all paths and just assign that score to this candidate/frame combo
	score->scores[frame_i] = best_score;
}



void score_candidates_against_frames(int rank, score *scores, int num_candidates, candidate *candidates, int num_pointcloud_frames, frame *pointcloud_frames, int num_skeleton_frames, frame *skeleton_frames){
	
	for(int candidate_i = 0; candidate_i < num_candidates; candidate_i++){
		
		memcpy(&(scores[candidate_i].candidate), &(candidates[candidate_i]), sizeof(candidate));
		scores[candidate_i].scores = (double *) malloc (num_pointcloud_frames * sizeof(double));
		
		for(int frame_i = 0; frame_i < num_pointcloud_frames; frame_i++){
			
			score_candidates_against_frame(&(scores[candidate_i]), frame_i, &(pointcloud_frames[frame_i]), &(skeleton_frames[frame_i]));
			printf("%i scoring candidate %i against frame %i >> %f\n", rank, candidate_i, frame_i, scores[candidate_i].scores[frame_i]);
			
//			break;
		}
//		break;
	}
	
}




bool is_leader(int rank){
	return 0 == rank;
}

int main(int argc, char** argv) {
	MPI_Init(NULL, NULL);
	
	srand(time(NULL));
	
	int world_size;
	MPI_Comm_size(MPI_COMM_WORLD, &world_size);
	
	int rank;
	MPI_Comm_rank(MPI_COMM_WORLD, &rank);
	
//	printf("Hello world from rank %d out of %d processors\n", rank, world_size);
	
	if(argc < EXPECTED_ARG_COUNT){
		if(is_leader(rank)){
			printf("Usage: ./attachment skeleton.csv pointcloud.csv\n");
		}
		return 1;
	}else if(world_size < MIN_PROCESSORS){
		if(is_leader(rank)){
			printf("System requires at least %i cores\n", MIN_PROCESSORS);
		}
		return 1;
	}
	
	int num_skeleton_frames = 0;
	frame *all_skeleton_frames = NULL;
	point *skeleton_packed_points = NULL;
	
	int num_pointcloud_frames = 0;
	frame *all_pointcloud_frames = NULL;
	point *pointcloud_packed_points = NULL;
	
	
	if(is_leader(rank)){
		read_and_broadcast_frames(argv, &num_skeleton_frames, &all_skeleton_frames, &num_pointcloud_frames, &all_pointcloud_frames);
	}else{
		listen_for_frames(rank, &num_skeleton_frames, &all_skeleton_frames, &skeleton_packed_points, &num_pointcloud_frames, &all_pointcloud_frames, &pointcloud_packed_points);
		
		// compute candidate for first num_skeleton_frames / world_size
		
	}
	
	if(num_pointcloud_frames != num_skeleton_frames){
		if(is_leader(rank)){
			printf("Invalid frame counts, unequal. %i PCL, %i skeleton\n", num_pointcloud_frames, num_skeleton_frames);
		}
		exit(1);
	}
	
	int worker_count = world_size - 1;
	int min_batch_size = num_skeleton_frames / worker_count;
	int batch_left_over = num_skeleton_frames - (min_batch_size * worker_count);
	
	int *frames_per_worker = (int *) malloc(world_size * sizeof(int));
	int *batch_start = (int *) malloc(world_size * sizeof(int));
	memset(frames_per_worker, 0, world_size * sizeof(int));
	int total_assigned = 0;
	for(int i = 1; i < world_size; i++){
		frames_per_worker[i] = min_batch_size;
		batch_start[i] = (i == 0) ? 0 : batch_start[i - 1] + frames_per_worker[i - 1];
		
		if(batch_left_over > 0 && i <= batch_left_over){
			frames_per_worker[i]++;
		}
	}
	int my_batch_size = frames_per_worker[rank];
	int my_batch_start = batch_start[rank];
	printf("> %i my_batch_size: %i, starting at %i\n",rank, my_batch_size, my_batch_start);
	
	// unnecessary, but used for log ordering
	MPI_Barrier(MPI_COMM_WORLD);
	
	int *candidates_per_worker = NULL;
	int *candidates_per_worker_in_bytes = NULL;
	int *candidates_per_worker_displacement = NULL;
	candidate *candidates = NULL;
	int num_candidates = 0;
	
	if(is_leader(rank)){
		// receive candidates
		candidates_per_worker = (int *) malloc (world_size * sizeof(int));
		candidates_per_worker_in_bytes = (int *) malloc (world_size * sizeof(int));
		candidates_per_worker_displacement = (int *) malloc (world_size * sizeof(int));
		
		MPI_Gather(&my_batch_size, 1, MPI_INT, candidates_per_worker, 1, MPI_INT, 0, MPI_COMM_WORLD);
		
		printf("ROOT: received candidate counts:\n");
		
		for(int i = 0; i < world_size; i++){
			candidates_per_worker_displacement[i] = num_candidates * sizeof(candidate);
			num_candidates += candidates_per_worker[i];
			candidates_per_worker_in_bytes[i] = candidates_per_worker[i] * sizeof(candidate);
			printf("ROOT: From %i: %i for %i bytes\n", i, candidates_per_worker[i], candidates_per_worker_in_bytes[i]);
		}
		
		candidates = (candidate *) malloc (num_candidates * sizeof(candidate));
		memset(candidates, 0, num_candidates * sizeof(candidate));
		
		MPI_Gatherv(NULL, 0, MPI_BYTE, candidates, candidates_per_worker_in_bytes, candidates_per_worker_displacement, MPI_BYTE, 0, MPI_COMM_WORLD);
		printf("ROOT: received %i candidates via gatherv\n", num_candidates);
		
		validate_candidates(rank, num_candidates, candidates);
		
		printf("ROOT: all candidates valid\n");
		
	}else{
		// compute candidates
		compute_candidate_for_frames(rank, my_batch_size, my_batch_start, &(all_skeleton_frames[my_batch_start]));
		
	}
	MPI_Bcast(&num_candidates, 1, MPI_INT, 0, MPI_COMM_WORLD);
	
	// Now rank0 has all candidates for all frames
	int min_candidate_batch_size = num_candidates / worker_count;
	int candidate_batch_left_over = num_candidates - (min_candidate_batch_size * worker_count);
	
	int *num_candidates_per_worker = (int *) malloc(world_size * sizeof(int));
	int *num_candidates_per_worker_in_bytes = (int *) malloc(world_size * sizeof(int));
	int *num_candidates_per_worker_displacement = (int *) malloc(world_size * sizeof(int));
	int *candidates_batch_start = (int *) malloc(world_size * sizeof(int));
	
	memset(num_candidates_per_worker_in_bytes, 0, world_size * sizeof(int));
	memset(num_candidates_per_worker_displacement, 0, world_size * sizeof(int));
	memset(num_candidates_per_worker, 0, world_size * sizeof(int));
	memset(candidates_batch_start, 0, world_size * sizeof(int));
	
	int total_candidates_assigned = 0;
	for(int i = 1; i < world_size; i++){
		num_candidates_per_worker[i] = min_candidate_batch_size;
		
		candidates_batch_start[i] = (i == 0) ? 0 : candidates_batch_start[i - 1] + num_candidates_per_worker[i - 1];
		num_candidates_per_worker_displacement[i] = candidates_batch_start[i] * sizeof(candidate);
		
		if(candidate_batch_left_over > 0 && i <= candidate_batch_left_over){
			num_candidates_per_worker[i]++;
		}
		
		num_candidates_per_worker_in_bytes[i] = num_candidates_per_worker[i] * sizeof(candidate);
	}
	int my_candidate_batch_size = num_candidates_per_worker[rank];
	int my_candidate_batch_start = candidates_batch_start[rank];

	printf("> %i my_candidate_batch_size: %i, starting at %i\n",rank, my_candidate_batch_size, my_candidate_batch_start);
	if(!is_leader(rank)){
		candidates = (candidate *) malloc (num_candidates_per_worker_in_bytes[rank]);
	}
	
	MPI_Scatterv(candidates, num_candidates_per_worker_in_bytes, num_candidates_per_worker_displacement, MPI_BYTE, candidates, num_candidates_per_worker_in_bytes[rank], MPI_BYTE, 0, MPI_COMM_WORLD);
	
	validate_candidates(rank, my_candidate_batch_size, candidates);
	printf("%i PASSED candidate validation\n", rank);
	
	MPI_Barrier(MPI_COMM_WORLD);
	
	// now each worker has (1/n)th of the candidates and all the frames.
	// time to score my candidates against all frames
	// one score per candidate, as it will have its scores for each frame internally
	score *scores = (score *) malloc (my_candidate_batch_size * sizeof(score));
	
	score_candidates_against_frames(rank, scores, my_candidate_batch_size, candidates, num_pointcloud_frames, all_pointcloud_frames, num_skeleton_frames, all_skeleton_frames);

	
	MPI_Barrier(MPI_COMM_WORLD);
	
	
	printf("> %i done\n", rank);
	if(scores){
		free(scores);
	}
	if(num_candidates_per_worker_displacement){
		free(num_candidates_per_worker_displacement);
	}
	if(num_candidates_per_worker_in_bytes){
		free(num_candidates_per_worker_in_bytes);
	}
	if(candidates){
		free(candidates);
	}
	if(candidates_per_worker_displacement){
		free(candidates_per_worker_displacement);
	}
	if(candidates_per_worker_in_bytes){
		free(candidates_per_worker_in_bytes);
	}
	if(candidates_per_worker){
		free(candidates_per_worker);
	}
	if(frames_per_worker){
		free(frames_per_worker);
	}
	if(all_skeleton_frames){
		free(all_skeleton_frames);
	}
	if(all_pointcloud_frames){
		free(all_pointcloud_frames);
	}
	if(pointcloud_packed_points){
		free(pointcloud_packed_points);
	}
	if(skeleton_packed_points){
		free(skeleton_packed_points);
	}
	MPI_Finalize();
}
