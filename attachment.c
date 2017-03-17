#include <mpi.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define MIN_PROCESSORS 4

#define MAX_EDGES 5

#define SAMPLE_PCL_POINTS 1
// rate / 100 ~= sample%
#define PCL_POINTS_SAMPLE_RATE 30

#define FRAME_DELIMITER '='

#define EXPECTED_ARG_COUNT 3

#include "util.h"


bool is_leader(int rank){
	return 0 == rank;
}

void read_frames(FILE *file_handle, frame_type type, int *num_frames, frame **all_frames){
	
	if((*all_frames)){
		free((*all_frames));
	}
	frame frm;
	
	(*all_frames) = NULL;
	int space_for_frames = 0;
	(*all_frames) = get_more_space_and_copy(&space_for_frames, (*all_frames), (*num_frames), type, sizeof(frame));
	
	
	while(true){
		memset(&frm, 0, sizeof(frame));
		read_frame(file_handle, &frm, type);
		
		if(frm.num_points > 0){
			(*all_frames) = get_more_space_and_copy(&space_for_frames, (*all_frames), (*num_frames), type, sizeof(frame));
			memcpy(&((*all_frames)[(*num_frames)]), &frm, sizeof(frame));
//			printf("%i read frame with %i points\n", (*num_frames), (*all_frames)[(*num_frames)].num_points);
			(*num_frames)++;
		}else{
			// break once we hit a frame with no points.
			break;
		}
	}
}

void pack_and_send(int num_frames, frame *frames){
	// send number of frames
	MPI_Bcast(&num_frames, 1, MPI_INT, 0, MPI_COMM_WORLD);
	
	/// collect number of points, send
	
	int point_count = 0;
	for(int i = 0; i < num_frames; i++){
		point_count += frames[i].num_points;
	}
//	printf("total points: %i\n", point_count);
	
	MPI_Bcast(&point_count, 1, MPI_INT, 0, MPI_COMM_WORLD);
//	printf("copied the fact that its %i points\n", point_count);
	
	
	// pack points and send
	int *points_per_frame = (int *) malloc (point_count * sizeof(int));
	point *points = (point *) malloc (point_count * sizeof(point));
	int point_i = 0;
	for(int i = 0; i < num_frames; i++){
		memcpy(&(points[point_i]), frames[i].points, frames[i].num_points * sizeof(point));
		points_per_frame[i] = frames[i].num_points;
		point_i += frames[i].num_points;
	}
	MPI_Bcast(points, point_count * 3, MPI_DOUBLE, 0, MPI_COMM_WORLD);
//	printf("packed/sent %i frame points\n", point_count);
	
	
	// send number of points per frame
	MPI_Bcast(points_per_frame, point_count, MPI_INT, 0, MPI_COMM_WORLD);
	
	
}

void receive_and_unpack(int *num_frames, frame **frames, point **points){
	// get number of frames
	MPI_Bcast(num_frames, 1, MPI_INT, 0, MPI_COMM_WORLD);
	(*frames) = (frame *) malloc ((*num_frames) * sizeof(frame));
	
	// get total number of packed points, make room
	int total_points;
	MPI_Bcast(&total_points, 1, MPI_INT, 0, MPI_COMM_WORLD);
	(*points) = (point *) malloc (total_points * sizeof(point));
	
	// get packed points
	MPI_Bcast(*points, total_points * 3, MPI_DOUBLE, 0, MPI_COMM_WORLD);
//	printf("Received %i points\n", total_points);
	int index = 10;
//	printf("%f,%f,%f\n", (*points)[index].x,(*points)[index].y,(*points)[index].z);
	
	// get counts, break up frame points
	int *points_per_frame = (int *) malloc (total_points * sizeof(int));
	MPI_Bcast(points_per_frame, total_points, MPI_INT, 0, MPI_COMM_WORLD);
//	printf("received that there are %i frames with points\n", total_points);
//	printf("%i, %i, %i\n", points_per_frame[0],points_per_frame[1],points_per_frame[2]);
	
	// build frames
	int point_i = 0;
	
	for(int i = 0; i < (*num_frames); i++){
//		printf("point_i = %i, points_per_frame[i] = %i\n", point_i, points_per_frame[i]);
		
		(*frames)[i].num_points = points_per_frame[i];
		(*frames)[i].points = &((*points)[point_i]);
		
		point_i += points_per_frame[i];
	}
	
}


void read_and_broadcast_frames(char **argv, int *num_skeleton_frames, frame **all_skeleton_frames, int *num_pointcloud_frames, frame **all_pointcloud_frames){
	
	FILE *skeleton_handle;
	FILE *pcl_handle;
	
	char * input_skeleton_file = argv[1];
	char * input_pcl_file = argv[2];
	
	skeleton_handle = fopen(input_skeleton_file, "r");
	pcl_handle = fopen(input_pcl_file, "r");
	
	read_frames(skeleton_handle, SKELETON, num_skeleton_frames, all_skeleton_frames);
	printf("done reading in %i skeleton frames\n", *num_skeleton_frames);
	
	read_frames(pcl_handle, POINTCLOUD, num_pointcloud_frames, all_pointcloud_frames);
	printf("done reading in %i pointcloud frames\n", *num_pointcloud_frames);
	
	MPI_Bcast(num_skeleton_frames, 1, MPI_INT, 0, MPI_COMM_WORLD);
	MPI_Bcast(num_pointcloud_frames, 1, MPI_INT, 0, MPI_COMM_WORLD);
	
	pack_and_send(*num_skeleton_frames, *all_skeleton_frames);
	pack_and_send(*num_pointcloud_frames, *all_pointcloud_frames);
	
	if(skeleton_handle){
		fclose(skeleton_handle);
	}
}

void listen_for_frames(int rank, int *num_skeleton_frames, frame **all_skeleton_frames, point **skeleton_packed_points, int *num_pointcloud_frames, frame **all_pointcloud_frames, point **pointcloud_packed_points){
	MPI_Bcast(num_skeleton_frames, 1, MPI_INT, 0, MPI_COMM_WORLD);
	MPI_Bcast(num_pointcloud_frames, 1, MPI_INT, 0, MPI_COMM_WORLD);
//	printf("%i > %i skeleton frames, %i pointcloud frames\n", rank, *num_skeleton_frames, *num_pointcloud_frames);
	
	receive_and_unpack(num_skeleton_frames, all_skeleton_frames, skeleton_packed_points);
	printf("> %i Finished rebuilding %i skeleton frames\n", rank, *num_skeleton_frames);
	receive_and_unpack(num_pointcloud_frames, all_pointcloud_frames, pointcloud_packed_points);
	printf("> %i Finished rebuilding %i pointcloud frames\n", rank, *num_pointcloud_frames);
	
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
	memset(frames_per_worker, 0, world_size * sizeof(int));
	int total_assigned = 0;
	for(int i = 1; i < world_size; i++){
		frames_per_worker[i] = min_batch_size;
		if(min_batch_size * worker_count < num_skeleton_frames && i > min_batch_size * worker_count){
			frames_per_worker[i]++;
		}
	}
	int my_batch_size = frames_per_worker[rank];
	printf("> %i my_batch_size: %i\n",rank, my_batch_size);
	
	// compute candidates
	
	
	
	
	
	
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
