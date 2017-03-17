#include <mpi.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>


#define PRECISION_DIGITS 3
#define MAX_EDGES 5
#define MAX_PCL_POINTS 100

#define FRAME_DELIMITER '='

#define EXPECTED_ARG_COUNT 3

typedef enum { SKELETON, POINTCLOUD } frame_type;

typedef enum { false, true } bool;

typedef struct a_point{
	double x;
	double y;
	double z;
} point;

typedef struct point_frame {
	int num_points;
	point *points;
} frame;

point * get_more_space_and_copy(int *space_for, point *points, int so_far, frame_type type){
	
	if((*space_for) > so_far){
		return points;
	}
	int to_allocate = so_far > 0 ? so_far * 2 : (type == SKELETON ? 16 : 64);
	
	point *temp = (point *) malloc (to_allocate * sizeof(point));
	
	if(so_far > 0){
		memcpy(temp, points, so_far * sizeof(point));
	}
	(*space_for) = to_allocate;
	
	return temp;
}

void read_frame(FILE *handle, frame *frm, frame_type type){
	char * line = NULL;
	size_t len = 0;
	ssize_t read;
	
	
	frm->num_points = 0;
	
	
	int space_for_points = 0;
	if(frm->points){
		free(frm->points);
	}
	frm->points = get_more_space_and_copy(&space_for_points, frm->points, frm->num_points, type);
	point *current;
	
	while ((read = getline(&line, &len, handle)) != -1) {
//		printf("Retrieved line of length %zu :\n", read);
//		printf("%s", line);
		if(FRAME_DELIMITER == line[0]){
			printf("%i\n", frm->num_points);
			break;
		}
		frm->points = get_more_space_and_copy(&space_for_points, frm->points, frm->num_points, type);
		current = &(frm->points[frm->num_points]);
		sscanf(line, "%lf,%lf,%lf", &(current->x), &(current->y), &(current->z));
		printf("READ LINE: %lf, %lf, %lf\n", current->x, current->y, current->z);
		
		frm->num_points++;
	}
	
	if(line){
		free(line);
	}
}

bool is_leader(int rank){
	return 0 == rank;
}

void read_and_broadcast_frames(char **argv){
	frame frm;
	FILE *skeleton_handle;
	FILE *pcl_handle;
	
	char * input_skeleton_file = argv[1];
	char * input_pcl_file = argv[2];
	
	skeleton_handle = fopen(input_skeleton_file, "r");
	
	read_frame(skeleton_handle, &frm, SKELETON);
	
	
	if(skeleton_handle){
		fclose(skeleton_handle);
	}
}

void listen_for_frames(){
	frame frm;
	
	
}

int main(int argc, char** argv) {
	MPI_Init(NULL, NULL);
	
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
	}
	
	if(is_leader(rank)){
		read_and_broadcast_frames(argv);
	}else{
		listen_for_frames();
	}
	
	
	MPI_Barrier(MPI_COMM_WORLD);
	
	
	printf("> %i done\n", rank);

	MPI_Finalize();
}
