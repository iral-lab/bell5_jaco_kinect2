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
	
	
	
	if(skeleton_handle){
		fclose(skeleton_handle);
	}
}

void listen_for_frames(int rank, int *num_skeleton_frames, frame **all_skeleton_frames, int *num_pointcloud_frames, frame **all_pointcloud_frames){
	
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
	int num_pointcloud_frames = 0;
	frame *all_pointcloud_frames = NULL;
	
	if(is_leader(rank)){
		read_and_broadcast_frames(argv, &num_skeleton_frames, &all_skeleton_frames, &num_pointcloud_frames, &all_pointcloud_frames);
	}else{
		listen_for_frames(rank, &num_skeleton_frames, &all_skeleton_frames, &num_pointcloud_frames, &all_pointcloud_frames);
	}
	
	
	MPI_Barrier(MPI_COMM_WORLD);
	
	
	printf("> %i done\n", rank);

	MPI_Finalize();
}
