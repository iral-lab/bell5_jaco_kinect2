#include <mpi.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>

#define PRECISION_DIGITS 3
#define MAX_EDGES 5
#define MAX_PCL_POINTS 100

#define FRAME_DELIMITER "="

#define EXPECTED_ARG_COUNT 3

typedef struct a_point{
	double x;
	double y;
	double z;
} point;

typedef struct point_frame {
	int num_points;
	point *points;
} frame;

void read_skeleton_frame(FILE *handle, frame *frm){
	
	frm->num_points = 0;
	frm->points = NULL;
	
}


int main(int argc, char** argv) {
	// Initialize the MPI environment
	MPI_Init(NULL, NULL);
	
	// Get the number of processes
	int world_size;
	MPI_Comm_size(MPI_COMM_WORLD, &world_size);
	
	// Get the rank of the process
	int world_rank;
	MPI_Comm_rank(MPI_COMM_WORLD, &world_rank);
	
	// Get the name of the processor
	char processor_name[MPI_MAX_PROCESSOR_NAME];
	int name_len;
	MPI_Get_processor_name(processor_name, &name_len);
	
	// Print off a hello world message
	printf("Hello world from processor %s, rank %d"
		   " out of %d processors\n",
		   processor_name, world_rank, world_size);
	
	for(int i = 0; i < argc; i++){
		printf("Arg %i: %s\n", i, argv[i]);
	}
	
	if(argc < EXPECTED_ARG_COUNT){
		printf("Usage: ./attachment skeleton.csv pointcloud.csv\n");
		return 1;
	}
	
	char * input_skeleton_file = argv[1];
	char * input_pcl_file = argv[2];
	
	FILE *handle = fopen(input_skeleton_file, "r");
	
	frame frm;
	read_skeleton_frame(handle, &frm);
	
	
	// Finalize the MPI environment.
	MPI_Finalize();
}
