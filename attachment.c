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
	
//	if((*all_frames)){
//		free((*all_frames));
//	}
//	
//	(*all_frames) = NULL;
	frame frm;
	memset(&frm, 0, sizeof(frame));
	
	read_frame(file_handle, &frm, SKELETON);
	send_frame_to(&frm, 1);
	
}

void read_and_broadcast_frames(char **argv){
	
	FILE *skeleton_handle;
	FILE *pcl_handle;
	
	char * input_skeleton_file = argv[1];
	char * input_pcl_file = argv[2];
	
	skeleton_handle = fopen(input_skeleton_file, "r");
	pcl_handle = fopen(input_pcl_file, "r");
	
	int num_skeleton_frames = 0;
	frame *all_frames = NULL;
	read_frames(skeleton_handle, SKELETON, &num_skeleton_frames, &all_frames);
	
	
//	read_frame(pcl_handle, &frm, POINTCLOUD);
	
//	send_frame_to(&frm, 1);
	
	
	if(skeleton_handle){
		fclose(skeleton_handle);
	}
}

void listen_for_frames(int rank){
	frame frm;
	
	if(1 == rank){
		get_frame_from(&frm, 0);
	}
	
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
	
	if(is_leader(rank)){
		read_and_broadcast_frames(argv);
	}else{
		listen_for_frames(rank);
	}
	
	
	MPI_Barrier(MPI_COMM_WORLD);
	
	
	printf("> %i done\n", rank);

	MPI_Finalize();
}
