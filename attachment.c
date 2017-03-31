#include <mpi.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <assert.h>

#define SHORT_TEST_RUN 1

// define in terms of millimeters instead of meters
#define UNIT_SCALAR 1000

#define BIG_NUMBER 999999999

#define MIN_PROCESSORS 2

#define MAX_POINTS_IN_SKELETON_FRAME 15

#define MAX_EDGES 5
#define MAX_VERTICES MAX_EDGES + 1
#define MIN_VERTICES 3 // at least 2 edges


#define MAX_ANCHORS 3
#define BRANCH_NEIGHBORS 3

#define LAMBDA_SCALAR 1.1

#define SAMPLE_PCL_POINTS 1
// rate / 100 ~= sample%
#define PCL_POINTS_SAMPLE_RATE 10

#define FRAME_DELIMITER '='

#define EXPECTED_ARG_COUNT 5

#include "util.h"
#include "compute_candidates.h"

typedef struct scored_path{
	path path;
	unsigned int penalty;
}scored_path;

typedef struct score{
	candidate candidate;
	unsigned int *penalties; // will store each frame's score in order
	unsigned short num_penalties;
}score;

typedef struct final_score{
	candidate candidate;
	unsigned int penalty;
}final_score;

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


unsigned int get_error_to_path(stateless_path *path, frame *pcl_frame){
	unsigned int error = 0;
	
	point *p,*p0,*p1;
	unsigned int best_distance, this_distance;
	int i,j;
	for(i = 0; i < pcl_frame->num_points; i++){
		p = &(pcl_frame->points[i]);
		best_distance = BIG_NUMBER;
		
		for(j = 0; j < path->num_points - 1; j++){
			p0 = path->points[j];
			p1 = path->points[j+1];
			
			this_distance = distance_to_segment(p, p0, p1);
			
			if(this_distance < best_distance){
				best_distance = this_distance;
			}
		}
		
		// protect against overflow
		error = MAX(error, error + best_distance);
	}
	return error;
}

int sort_by_penalty(const void *a, const void *b){
	// returns lower scores as better
	return ((final_score *) a)->penalty < ((final_score *) b)->penalty ? -1 : 1;
}

unsigned int score_candidates_against_frame(score *score, int frame_i, frame *pcl_frame, frame *skeleton_frame){
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
	
	unsigned int best_score = BIG_NUMBER;
	unsigned int temp_score;
	
	point *best_point = NULL;
	unsigned int smallest_error = BIG_NUMBER;
	unsigned int best_points_distance = 0;
	unsigned int error;
	
	clock_t start, diff;
	clock_t all_start = clock(), all_diff;
	int scoring_time_spent = 0;
	int total_time_spent = 0;
	while(stack_size > 0){
		memcpy(&current_path, &(stack[stack_size-1]), sizeof(stateless_path));
		stack_size--;
		
		if(current_path.num_points == max_path_vertices){
			start = clock();
			
			temp_score = get_error_to_path(&current_path, pcl_frame);
			
			diff = clock() - start;
			scoring_time_spent += diff * 1000 / CLOCKS_PER_SEC;
			
			if(temp_score < best_score){
				best_score = temp_score;
			}
			
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
			smallest_error = BIG_NUMBER;
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
	
	
	if(stack){
		free(stack);
	}
	
	return best_score;
}



unsigned int compute_final_penalty(unsigned int num_penalties, unsigned int *penalties){
	if(num_penalties == 0){
		return 0;
	}
	
	// basic, compute average.
	unsigned int sum = 0;
	int i;
	for(i = 0; i < num_penalties; i++){
//		printf("sum %i, num %i val %i\n", sum, num_penalties, penalties[i]);
		sum += penalties[i];
	}
	return floorf(sum / num_penalties);
}

void score_candidates_against_frames(int rank, final_score *final_scores, int num_candidates, candidate *candidates, int num_pointcloud_frames, frame *pointcloud_frames, int num_skeleton_frames, frame *skeleton_frames){
	int candidate_i, frame_i;
	
	score score;
	memset(&score, 0, sizeof(score));
	
	score.penalties = (unsigned int *) malloc (num_pointcloud_frames * sizeof(unsigned int));
	
	unsigned int value;
	
	for(candidate_i = 0; candidate_i < num_candidates; candidate_i++){
		memset(score.penalties, 0, num_pointcloud_frames * sizeof(unsigned int));
		memcpy(&(score.candidate), &(candidates[candidate_i]), sizeof(candidate));
		
		score.num_penalties = 0;
		
		for(frame_i = 0; frame_i < num_pointcloud_frames; frame_i++){
			
			value = score_candidates_against_frame(&(score), frame_i, &(pointcloud_frames[frame_i]), &(skeleton_frames[frame_i]));
			
			if(value <= 0){
				printf("%i HAS ZERO OR NEG PENALTY: %i\n", rank, value);
				exit(1);
			}
			
			score.penalties[score.num_penalties] = value;
			score.num_penalties++;
			
			if(SHORT_TEST_RUN){
				break;
			}
		}
		memcpy(&(final_scores[candidate_i].candidate), &(candidates[candidate_i]), sizeof(candidate));
		final_scores[candidate_i].penalty = compute_final_penalty(score.num_penalties, score.penalties);
		
		printf("%i scoring candidate %i/%i (%i edges) = %i\n", rank, candidate_i, num_candidates, score.candidate.num_lengths, final_scores[candidate_i].penalty);
		fflush(stdout);
		
		if(SHORT_TEST_RUN){
			break;
		}
	}
	
	free(score.penalties);
}



bool is_leader(int rank){
	return 0 == rank;
}

void get_best_candidate_from_output(candidate *best_cand, int best_length, char *output_file){
	FILE *output_handle = fopen(output_file, "r");
	candidate cand;
	
	if(!output_handle){
		printf("Output file does not exist: %s\n", output_file);
		exit(1);
	}
	
	char * line = NULL;
	size_t len = 0;
	ssize_t read;
	
	unsigned int score;
	unsigned int best_score = BIG_NUMBER;
	
	unsigned int num_lengths;
	unsigned int total_length;
	unsigned int length_0;
	unsigned int length_1;
	unsigned int length_2;
	unsigned int length_3;
	unsigned int length_4;
	
	// the output-file reader needs to be correct, this was written to have this number
	assert(5 == MAX_EDGES);
	
	while ((read = getline(&line, &len, output_handle)) != -1) {
		//			printf("line: %s\n", line);
		sscanf(line, "%i,%i,%i,%i,%i,%i,%i,%i", &score, &(num_lengths), &(total_length), &(length_0), &(length_1), &(length_2), &(length_3), &(length_4));
		
		cand.num_lengths = (unsigned short) num_lengths;
		cand.total_length = (unsigned short) total_length;
		cand.lengths[0] = (unsigned short) length_0;
		cand.lengths[1] = (unsigned short) length_1;
		cand.lengths[2] = (unsigned short) length_2;
		cand.lengths[3] = (unsigned short) length_3;
		cand.lengths[4] = (unsigned short) length_4;
		
		//			printf("READ LINE: %i, %i \n", score, cand.num_lengths);
		
		if(cand.num_lengths == best_length && score < best_score){
			memset(best_cand, 0, sizeof(candidate));
			memcpy(best_cand, &cand, sizeof(candidate));
			best_score = score;
			printf("NEW BEST: %i, %hu, %hu \n", best_score, best_cand->total_length, best_cand->num_lengths);
		}
		
	}
	
	if(output_handle){
		fclose(output_handle);
	}
}

void do_best_robot_output(int rank, unsigned short best_length, int my_batch_size, int my_batch_start, int num_skeleton_frames, frame *all_skeleton_frames, int num_pointcloud_frames, frame *all_pointcloud_frames, char *output_file){
	
	candidate cand;
	memset(&cand,0,sizeof(candidate));
	
	if(is_leader(rank)){
		get_best_candidate_from_output(&cand, best_length, output_file);
		printf("best cand: %i\n", cand.total_length);
	}
	MPI_Bcast(&cand, sizeof(candidate), MPI_BYTE, 0, MPI_COMM_WORLD);
	
	printf("%i received cand: %i\n", rank, cand.total_length);
	
	
	
	MPI_Barrier(MPI_COMM_WORLD);
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
			printf("Usage: ./attachment run|(best n) skeleton.csv pointcloud.csv output.csv\n");
		}
		return 1;
	}else if(world_size < MIN_PROCESSORS){
		if(is_leader(rank)){
			printf("System requires at least %i cores\n", MIN_PROCESSORS);
		}
		return 1;
	}
	
	int i,j;
	
	int num_skeleton_frames = 0;
	frame *all_skeleton_frames = NULL;
	point *skeleton_packed_points = NULL;
	
	int num_pointcloud_frames = 0;
	frame *all_pointcloud_frames = NULL;
	point *pointcloud_packed_points = NULL;
	
	char * input_skeleton_file;
	char * input_pcl_file;
	char * output_file;
	
	bool output_best = false;
	unsigned short best_length = 0;
	
	if(strcmp(argv[1], "run") == 0){
		input_skeleton_file = argv[2];
		input_pcl_file = argv[3];
		output_file = argv[4];
	}else if(strcmp(argv[1], "best") == 0){
		output_best = true;
		best_length = atoi(argv[2]);
		if(best_length == 0){
			printf("Invalid best length: %s\n", argv[2]);
			exit(1);
		}
		input_skeleton_file = argv[3];
		input_pcl_file = argv[4];
		output_file = argv[5];
	}else{
		printf("Invalid run-type stated: %s\n", argv[1]);
		exit(1);
	}
	
	if(is_leader(rank)){
		read_and_broadcast_frames(input_skeleton_file, input_pcl_file, &num_skeleton_frames, &all_skeleton_frames, &num_pointcloud_frames, &all_pointcloud_frames);
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
	fflush(stdout);
	
	if(output_best){
		do_best_robot_output(rank, best_length, my_batch_size, my_batch_start, num_skeleton_frames, all_skeleton_frames, num_pointcloud_frames, all_pointcloud_frames, output_file);
		exit(0);
	}
	
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
		fflush(stdout);
		
		for(i = 0; i < world_size; i++){
			candidates_per_worker_displacement[i] = num_candidates * sizeof(candidate);
			num_candidates += candidates_per_worker[i];
			candidates_per_worker_in_bytes[i] = candidates_per_worker[i] * sizeof(candidate);
			if(candidates_per_worker[i] > 0){
				printf("ROOT: From %i: %i for %i bytes\n", i, candidates_per_worker[i], candidates_per_worker_in_bytes[i]);
			}
		}
		
		candidates = (candidate *) malloc (num_candidates * sizeof(candidate));
		memset(candidates, 0, num_candidates * sizeof(candidate));
		
		MPI_Gatherv(NULL, 0, MPI_BYTE, candidates, candidates_per_worker_in_bytes, candidates_per_worker_displacement, MPI_BYTE, 0, MPI_COMM_WORLD);
		printf("ROOT: received %i candidates via gatherv\n", num_candidates);
		fflush(stdout);
		
		validate_candidates(rank, num_candidates, candidates);
		
		printf("ROOT: all candidates valid\n");
		fflush(stdout);
		
//		 removing final de-dupe since with 500k candidates this is REALLY expensive.
//		It is good enough to have deduping on workers beforehand and after, to avoid this big cross
//		de-dupe candidates, as some frames might have same overlap, which would be good
//		deduplicate_candidates(rank, &num_candidates, candidates, 0, true);
//		
//		printf("ROOT: deduping done\n");
//		fflush(stdout);
		
		printf("ROOT: randomizing candidates\n");
		fflush(stdout);
		randomize_array(candidates, num_candidates, sizeof(candidate));
		
		validate_candidates(rank, num_candidates, candidates);
		printf("ROOT: all candidates valid\n");
		fflush(stdout);
		
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

//	printf("> %i my_candidate_batch_size: %i, starting at %i\n",rank, my_candidate_batch_size, my_candidate_batch_start);
	if(!is_leader(rank)){
		candidates = (candidate *) malloc (num_candidates_per_worker_in_bytes[rank]);
	}
	
	MPI_Scatterv(candidates, num_candidates_per_worker_in_bytes, num_candidates_per_worker_displacement, MPI_BYTE, candidates, num_candidates_per_worker_in_bytes[rank], MPI_BYTE, 0, MPI_COMM_WORLD);
	
	validate_candidates(rank, my_candidate_batch_size, candidates);
//	printf("%i PASSED candidate validation\n", rank);
	
	double total_length_count = 0;
	for(i = 0; i < my_candidate_batch_size; i++){
		total_length_count += candidates[i].num_lengths;
	}
//	printf("%i length count: %i / %i = %f\n", rank, (int)total_length_count, my_candidate_batch_size, (my_candidate_batch_size > 0 ? total_length_count / my_candidate_batch_size : 0.0));
	
	MPI_Barrier(MPI_COMM_WORLD);
	
	// now each worker has (1/n)th of the candidates and all the frames.
	// time to score my candidates against all frames
	// one score per candidate, as it will have its scores for each frame internally
	final_score *final_scores;
	
	if(is_leader(rank)){
		final_scores = (final_score *) malloc (num_candidates * sizeof(final_score));
		memset(final_scores, 0, num_candidates * sizeof(final_score));
	}else{
//		printf("%i doing pre-score deduping\n",rank);
		deduplicate_candidates(rank, &my_candidate_batch_size, candidates, 0, false);
		
		final_scores = (final_score *) malloc (my_candidate_batch_size * sizeof(final_score));
		memset(final_scores, 0, my_candidate_batch_size * sizeof(final_score));
		clock_t start = clock(), diff;
		score_candidates_against_frames(rank, final_scores, my_candidate_batch_size, candidates, num_pointcloud_frames, all_pointcloud_frames, num_skeleton_frames, all_skeleton_frames);
		
		diff = clock() - start;
		int msec = diff * 1000 / CLOCKS_PER_SEC;
		printf("%i Scoring taken %d seconds %d milliseconds\n", rank, msec/1000, msec%1000);
	}
	
	
	printf("%i finished scoring\n", rank); fflush(stdout);
	fflush(stdout);
	
	MPI_Gatherv(final_scores, my_candidate_batch_size, MPI_BYTE, final_scores, final_scores_per_worker_bytes, final_scores_per_worker_displacement, MPI_BYTE, 0, MPI_COMM_WORLD);
	
	if(is_leader(rank)){
		candidate *cand;
		printf("Received final scores.\n");
		
		qsort(final_scores, num_candidates, sizeof(final_score), sort_by_penalty);
		
		compute_candidate_total_lengths(final_scores, num_candidates);
		
		printf("final scores sorted\n");
		
		FILE *output_handle = fopen(output_file, "w");
		fprintf(output_handle, "score,num_edges,total_length_mm");
		for(i = 0; i < MAX_EDGES; i++){
			fprintf(output_handle,",length_%i",i);
		}
		fprintf(output_handle, "\n");
		
		for(i = 0; i < num_candidates; i++){
			if(final_scores[i].penalty == 0){
				continue;
			}
			cand = &(final_scores[i].candidate);
//			printf("cand %i => %f\n", i, final_scores[i].score);
			fprintf(output_handle, "%i,%i,%i", final_scores[i].penalty, cand->num_lengths, cand->total_length);
			
			for(j = 0; j < cand->num_lengths; j++){
				fprintf(output_handle, ",%i", cand->lengths[j]);
			}
			// pad out lengths to keep alignment consistent
			for(;j < MAX_EDGES; j++){
				fprintf(output_handle, ",0");
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
