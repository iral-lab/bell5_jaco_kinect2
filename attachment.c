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

void send_frame_to(frame *frm, int destination){
	MPI_Send(&(frm->num_points), 1, MPI_INT, destination, SEND_NUM_POINTS, MPI_COMM_WORLD);
	MPI_Send(frm->points, frm->num_points * 3, MPI_DOUBLE, destination, SEND_POINTS, MPI_COMM_WORLD);
}

void get_frame_from(frame *frm, int source){
	MPI_Recv(&(frm->num_points), 1, MPI_INT, source, SEND_NUM_POINTS, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
	frm->points = (point *) malloc (frm->num_points * sizeof(point));
	MPI_Recv(frm->points, frm->num_points * 3, MPI_DOUBLE, source, SEND_POINTS, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
}

void read_and_broadcast_frames(char **argv){
	frame frm;
	FILE *skeleton_handle;
	FILE *pcl_handle;
	
	char * input_skeleton_file = argv[1];
	char * input_pcl_file = argv[2];
	
	skeleton_handle = fopen(input_skeleton_file, "r");
	pcl_handle = fopen(input_pcl_file, "r");
	
	read_frame(skeleton_handle, &frm, SKELETON);
//	read_frame(pcl_handle, &frm, POINTCLOUD);
	
	send_frame_to(&frm, 1);
	
	
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
