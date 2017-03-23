#include <mpi.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>


#define MIN_PROCESSORS 4

#define MAX_EDGES 5

#define SAMPLE_PCL_POINTS 1
// rate / 100 ~= sample%
#define PCL_POINTS_SAMPLE_RATE 30

#define FRAME_DELIMITER '='

#define EXPECTED_ARG_COUNT 3

#include "util.h"


double euclid_distance(point *p0, point *p1){
	double diff_x = p0->x - p1->x;
	double diff_y = p0->y - p1->y;
	double diff_z = p0->z - p1->z;
	
	return sqrt( (diff_x * diff_x) + (diff_y * diff_y) + (diff_z * diff_z));
}

typedef struct sort_pair{
	int i;
	int j;
	double val;
}sort_pair;

int sort_pairwise_distances(const void *a, const void *b){
//	printf("COMPARING %f and %f\n",((sort_pair *)a)->val,((sort_pair *)b)->val);
	return ((sort_pair *)a)->val > ((sort_pair *)b)->val ? 1 : -1;
}

void get_pairwise_distances(int rank, int num_points, sort_pair **pairs, frame *frm){
	(*pairs) = (sort_pair *) malloc (num_points * num_points * sizeof(sort_pair));
	
	int offset;
	point *p0, *p1;
	for(int i = 0; i < num_points; i++){
		p0 = &(frm->points[i]);
		for(int j = 0; j < num_points; j++){
			offset = (i * num_points + j); // i = rows, j = cols
			(*pairs)[offset].i = i;
			(*pairs)[offset].j = j;
			p1 = &(frm->points[j]);
			if(i == j){
				(*pairs)[offset].val = 0.0;
			}else{
				(*pairs)[offset].val = euclid_distance(p0, p1);
			}
//			printf("1) Dist bet %f,%f,%f and %f,%f,%f = %f\n", p0->x, p0->y, p0->z, p1->x, p1->y, p1->z, (*pairs)[offset].val);
		}
		
		// got distances, now sort them inside each row
//		printf("> %i got distances\n", rank);
		qsort(&((*pairs)[i * num_points]), num_points, sizeof(sort_pair), sort_pairwise_distances);
//		printf("> %i sorted\n", rank);
		for(int j = 0; j < num_points; j++){
			p1 = &(frm->points[j]);
//			printf("< %i p1: %f %f %f\n", j, p1->x, p1->y, p1->z);
			offset = (i * num_points + j); // i = rows, j = cols
//			printf("offset: %i\n",offset);
			(*pairs)[offset].i = i;
			(*pairs)[offset].j = j;
//			printf("2) Dist bet %f,%f,%f and %f,%f,%f = %f\n", p0->x, p0->y, p0->z, p1->x, p1->y, p1->z, (*pairs)[offset].val);
		}
		
	}
}

void compute_candidates_for_frame(int rank, int frame_n, frame *frm){
	printf("> %i cand(f_%i)\n", rank, frame_n);
	
	int num_points = frm->num_points;
	sort_pair *pairs;
	get_pairwise_distances(rank, num_points, &pairs, frm);
	
	
	
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

	for(int i = 0; i < num_skeleton_frames; i++){
		compute_candidates_for_frame(rank, my_start + i, &(skeleton_frames[i]));
		break;
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
	
	int worker_count = world_size - 1;
	int min_batch_size = num_skeleton_frames / worker_count;
	
	int *frames_per_worker = (int *) malloc(world_size * sizeof(int));
	int *batch_start = (int *) malloc(world_size * sizeof(int));
	memset(frames_per_worker, 0, world_size * sizeof(int));
	int total_assigned = 0;
	for(int i = 1; i < world_size; i++){
		frames_per_worker[i] = min_batch_size;
//		if(rank == 0){
//			printf("%i --- %i, %i, %i, %i\n", i, min_batch_size, worker_count, min_batch_size * worker_count, num_skeleton_frames);
//		}
		batch_start[i] = (i == 0) ? 0 : batch_start[i - 1] + frames_per_worker[i - 1];
		if(i * min_batch_size < (num_skeleton_frames - i)){
			frames_per_worker[i]++;
		}
	}
	int my_batch_size = frames_per_worker[rank];
	int my_batch_start = batch_start[rank];
	printf("> %i my_batch_size: %i, starting at %i\n",rank, my_batch_size, my_batch_start);
	
	// unnecessary, but used for log ordering
	MPI_Barrier(MPI_COMM_WORLD);
	
	if(is_leader(rank)){
		// receive candidates
		
//	}else{
	}else if(rank == 1){
		// compute candidates
		
		compute_candidate_for_frames(rank, my_batch_size, my_batch_start, &(all_skeleton_frames[my_batch_start]));
		
	}
	
	
	
	
	MPI_Barrier(MPI_COMM_WORLD);
	
	
	printf("> %i done\n", rank);
	
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
