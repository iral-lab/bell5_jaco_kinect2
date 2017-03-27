#include <mpi.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>

#define SHORT_TEST_RUN 1

// define in terms of millimeters instead of meters
#define UNIT_SCALAR 1000

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

#define EXPECTED_ARG_COUNT 4

#include "util.h"
#include "compute_candidates.h"

typedef struct scored_path{
	path path;
	int score;
}scored_path;

typedef struct score{
	candidate candidate;
	int *scores; // will store each frame's score in order
	short num_scores;
}score;

typedef struct final_score{
	candidate candidate;
	int score;
}final_score;

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

void compute_candidate_total_lengths(final_score *final_scores, int num_candidates){
	candidate *candidate;
	int i,j;
	for(i = 0; i < num_candidates; i++){
		candidate = &(final_scores[i].candidate);
		candidate->total_length = 0.0;
		for(j = 0; j < candidate->num_lengths; j++){
			candidate->total_length += candidate->lengths[j];
			
		}
	}
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

int get_error_to_path(stateless_path *path, frame *pcl_frame){
	int error = 0;
	
	point *p,*p0,*p1;
	int best_distance, this_distance;
	int i,j;
	for(i = 0; i < pcl_frame->num_points; i++){
		p = &(pcl_frame->points[i]);
		best_distance = 999999999;
		
		for(j = 0; j < path->num_points - 1; j++){
			p0 = path->points[j];
			p1 = path->points[j+1];
			
			this_distance = floorf(distance_to_segment(p, p0, p1));
			
			if(this_distance < best_distance){
				best_distance = this_distance;
			}
		}
		
		error += best_distance;
	}
	return error;
}

int score_path(stateless_path *path, frame *pcl_frame){
	// want the error to be a penalty, more error => lower score
	return get_error_to_path(path, pcl_frame) * -1;
}

int sort_by_score(const void *a, const void *b){
	// returns higher scores as better
	return ((final_score *) a)->score < ((final_score *) b)->score ? 1 : -1;
}

void score_candidates_against_frame(score *score, int frame_i, frame *pcl_frame, frame *skeleton_frame){
	candidate *candidate = &(score->candidate);
	
	int num_points = skeleton_frame->num_points;
	
	// sort skeleton points by their y value
	qsort(skeleton_frame->points, skeleton_frame->num_points, sizeof(point), sort_by_y_value);
	int i,j,k;
	for(i = 0; i < num_points; i++){
		skeleton_frame->points[i].pid = i;
	}
	
	sort_pair *pairs;
	get_pairwise_distances(num_points, &pairs, skeleton_frame);
	
	int max_path_vertices = candidate->num_lengths + 1;
	short num_closest = get_num_closest(num_points);
	
	
	int space_on_stack = 0;
	int stack_size = 0;
	stateless_path *stack = initialize_stack_with_anchors_stateless(skeleton_frame, &stack_size, &space_on_stack);
	stateless_path current_path;
	point *last_point;
	point *other_point;
	point new_point;
	sort_pair *other_pair;
	
	int offset, added;
	int desired_length;
	
	bool added_pids[num_points];
	
	int best_score = -999999999;
	int temp_score;
	
	point *best_point = NULL;
	int smallest_error = 999999999;
	int best_points_distance = 0;
	int error;
	
	clock_t start, diff;
	clock_t all_start = clock(), all_diff;
	int scoring_time_spent = 0;
	int total_time_spent = 0;
	while(stack_size > 0){
		memcpy(&current_path, &(stack[stack_size-1]), sizeof(stateless_path));
		stack_size--;
		
		if(current_path.num_points == max_path_vertices){
			start = clock();
			
			temp_score = score_path(&current_path, pcl_frame);
			
			diff = clock() - start;
			scoring_time_spent += diff * 1000 / CLOCKS_PER_SEC;
			
			best_score = MAX(best_score, temp_score);
			
			continue;
		}
//		print_path(&current_path);
		
		last_point = current_path.points[ current_path.num_points - 1];
		added = 0;
		memset(added_pids, 0, num_points * sizeof(bool));
		desired_length = candidate->lengths[current_path.num_points-1];
		for(i = 0; i < num_closest && added < num_points; i++){
			
//			printf("looking for a point %i away from %i,%i,%i\n", desired_length, last_point->x, last_point->y, last_point->z);
			
			// adds the point that is the closest to the current length away from the last point
			
			
			best_point = NULL;
			smallest_error = 999999999;
			best_points_distance = 0;
			
			for(j = 1; j < num_points; j++){
				// start at 1 since 0 is itself
				offset = (last_point->pid * num_points + j);
				other_pair = &(pairs[offset]);
				
				if(added_pids[other_pair->pid]){
					continue;
				}
				
				error = abs(other_pair->distance - desired_length);
				if(error < smallest_error){
					other_point = &(skeleton_frame->points[other_pair->pid]);
					smallest_error = error;
					best_point = other_point;
					best_points_distance = other_pair->distance;
				}else{
					// once a point is even further away, it can't provide a lower error.
					break;
				}
			}
			
			if(!best_point){
				printf("SOMETHING WENT WRONG, no point was selected\n");
				exit(1);
			}
			
//			printf("adding pid %i at %i,%i,%i, which is %i away, error %i\n", best_point->pid, best_point->x, best_point->y, best_point->z, best_points_distance, smallest_error);
			
			// standard ending
			stack = get_more_space_and_copy(&space_on_stack, stack, stack_size, PATHS, sizeof(path));
			
			memcpy(&(stack[stack_size]), &current_path, sizeof(path));
			
			stack[stack_size].points[current_path.num_points] = best_point;
			stack[stack_size].num_points++;

			stack_size++;
			
			added_pids[best_point->pid] = true;
			added++;
			
		}
		
		
	}
	
	all_diff = clock() - all_start;
	total_time_spent += all_diff * 1000 / CLOCKS_PER_SEC;
	//printf("Scoring took %d seconds %d milliseconds\n", scoring_time_spent/1000, scoring_time_spent%1000);
	//printf("all took %d seconds %d milliseconds\n", total_time_spent/1000, total_time_spent%1000);
	
	// now find the best score among all paths and just assign that score to this candidate/frame combo
	score->scores[frame_i] = best_score;
}



int compute_final_score(int num_scores, int *scores){
	if(num_scores == 0){
		return 0;
	}
	
	// basic, compute average.
	int sum = 0;
	int i;
	for(i = 0; i < num_scores; i++){
		sum += scores[i];
	}
	return floorf(sum / num_scores);
}

void score_candidates_against_frames(int rank, final_score *final_scores, int num_candidates, candidate *candidates, int num_pointcloud_frames, frame *pointcloud_frames, int num_skeleton_frames, frame *skeleton_frames){
	int candidate_i, frame_i;
	
	score score;
	memset(&score, 0, sizeof(score));
	
	score.scores = (int *) malloc (num_pointcloud_frames * sizeof(int));
	
	
	for(candidate_i = 0; candidate_i < num_candidates; candidate_i++){
		memset(score.scores, 0, num_pointcloud_frames * sizeof(int));
		memcpy(&(score.candidate), &(candidates[candidate_i]), sizeof(candidate));
		
		score.num_scores = 0;
		
		printf("%i scoring candidate %i/%i (%i edges)\n", rank, candidate_i, num_candidates, score.candidate.num_lengths);
		
		for(frame_i = 0; frame_i < num_pointcloud_frames; frame_i++){
			
			score_candidates_against_frame(&(score), frame_i, &(pointcloud_frames[frame_i]), &(skeleton_frames[frame_i]));
			score.num_scores++;
			
			if(SHORT_TEST_RUN){
				break;
			}
		}
		memcpy(&(final_scores[candidate_i].candidate), &(candidates[candidate_i]), sizeof(candidate));
		final_scores[candidate_i].score = compute_final_score(score.num_scores, score.scores);
		
		if(SHORT_TEST_RUN){
			break;
		}
	}
	
	free(score.scores);
}


bool is_leader(int rank){
	return 0 == rank;
}

int main(int argc, char** argv) {
	MPI_Init(NULL, NULL);
	
//	srand(time(NULL));
	srand(1);
	
	int world_size;
	MPI_Comm_size(MPI_COMM_WORLD, &world_size);
	
	int rank;
	MPI_Comm_rank(MPI_COMM_WORLD, &rank);
	
//	printf("Hello world from rank %d out of %d processors\n", rank, world_size);
	
	if(argc < EXPECTED_ARG_COUNT){
		if(is_leader(rank)){
			printf("Usage: ./attachment skeleton.csv pointcloud.csv output.csv\n");
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
	int i,j;
	int worker_count = world_size - 1;
	int min_batch_size = num_skeleton_frames / worker_count;
	int batch_left_over = num_skeleton_frames - (min_batch_size * worker_count);
	
	int *frames_per_worker = (int *) malloc(world_size * sizeof(int));
	int *batch_start = (int *) malloc(world_size * sizeof(int));
	memset(frames_per_worker, 0, world_size * sizeof(int));
	memset(batch_start, 0, world_size * sizeof(int));
	
	int total_assigned = 0;
	for(i = 1; i < world_size; i++){
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
		
		for(i = 0; i < world_size; i++){
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
		
		// de-dupe candidates, as some frames might have same overlap, which would be good
		deduplicate_candidates(&num_candidates, candidates);
		
		printf("ROOT: deduping done\n");
		
		printf("ROOT: randomizing candidates\n");
		randomize_array(candidates, num_candidates, sizeof(candidate));
		
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
	
	int *final_scores_per_worker_bytes = (int *) malloc (world_size * sizeof(int));
	int *final_scores_per_worker_displacement = (int *) malloc (world_size * sizeof(int));
	memset(final_scores_per_worker_bytes, 0, world_size * sizeof(int));
	memset(final_scores_per_worker_displacement, 0, world_size * sizeof(int));
	
	
	
	int total_candidates_assigned = 0;
	for(i = 1; i < world_size; i++){
		num_candidates_per_worker[i] = min_candidate_batch_size;
		
		candidates_batch_start[i] = (i == 0) ? 0 : candidates_batch_start[i - 1] + num_candidates_per_worker[i - 1];
		num_candidates_per_worker_displacement[i] = candidates_batch_start[i] * sizeof(candidate);
		
		if(candidate_batch_left_over > 0 && i <= candidate_batch_left_over){
			num_candidates_per_worker[i]++;
		}
		
		final_scores_per_worker_bytes[i] = num_candidates_per_worker[i] * sizeof(final_score);
		final_scores_per_worker_displacement[i] = final_scores_per_worker_displacement[i-1] + final_scores_per_worker_bytes[i-1];
		
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
	
	double total_length_count = 0;
	for(i = 0; i < my_candidate_batch_size; i++){
		total_length_count += candidates[i].num_lengths;
	}
	printf("%i length count: %i / %i = %f\n", rank, (int)total_length_count, my_candidate_batch_size, (my_candidate_batch_size > 0 ? total_length_count / my_candidate_batch_size : 0.0));
	
	MPI_Barrier(MPI_COMM_WORLD);
	
	// now each worker has (1/n)th of the candidates and all the frames.
	// time to score my candidates against all frames
	// one score per candidate, as it will have its scores for each frame internally
	final_score *final_scores;
	
	if(is_leader(rank)){
		final_scores = (final_score *) malloc (num_candidates * sizeof(final_score));
		memset(final_scores, 0, num_candidates * sizeof(final_score));
	}else{
		final_scores = (final_score *) malloc (my_candidate_batch_size * sizeof(final_score));
		memset(final_scores, 0, my_candidate_batch_size * sizeof(final_score));
		clock_t start = clock(), diff;
		score_candidates_against_frames(rank, final_scores, my_candidate_batch_size, candidates, num_pointcloud_frames, all_pointcloud_frames, num_skeleton_frames, all_skeleton_frames);
		
		diff = clock() - start;
		int msec = diff * 1000 / CLOCKS_PER_SEC;
		printf("%i Scoring taken %d seconds %d milliseconds\n", rank, msec/1000, msec%1000);
	}
	
	
	printf("%i finished scoring\n", rank); fflush(stdout);
	
	MPI_Gatherv(final_scores, my_candidate_batch_size, MPI_BYTE, final_scores, final_scores_per_worker_bytes, final_scores_per_worker_displacement, MPI_BYTE, 0, MPI_COMM_WORLD);
	
	if(is_leader(rank)){
		candidate *cand;
		printf("Received final scores.\n");
		
		qsort(final_scores, num_candidates, sizeof(final_score), sort_by_score);
		
		compute_candidate_total_lengths(final_scores, num_candidates);
		
		printf("final scores sorted\n");
		
		char *output_file = argv[3];
		FILE *output_handle = fopen(output_file, "w");
		fprintf(output_handle, "score,num_edges,total_length_mm");
		for(i = 0; i < MAX_EDGES; i++){
			fprintf(output_handle,",length_%i",i);
		}
		fprintf(output_handle, "\n");
		
		for(i = 0; i < num_candidates; i++){
			if(final_scores[i].score == 0){
				continue;
			}
			cand = &(final_scores[i].candidate);
//			printf("cand %i => %f\n", i, final_scores[i].score);
			fprintf(output_handle, "%i,%i,%i", final_scores[i].score, cand->num_lengths, cand->total_length);
			
			for(j = 0; j < cand->num_lengths; j++){
				fprintf(output_handle, ",%i", cand->lengths[j]);
			}
			// pad out lengths to keep alignment consistent
			for(;j < MAX_EDGES; j++){
				fprintf(output_handle, ",0.0");
			}
			fprintf(output_handle, "\n");
		}
		if(output_handle){
			fclose(output_handle);
		}
		
	}
	
	MPI_Barrier(MPI_COMM_WORLD);
	
	printf("> %i done\n", rank);
	
	
	if(final_scores_per_worker_bytes){
		free(final_scores_per_worker_bytes);
	}
	if(final_scores_per_worker_displacement){
		free(final_scores_per_worker_displacement);
	}
	if(final_scores){
		free(final_scores);
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
