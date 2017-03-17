typedef enum { SKELETON, POINTCLOUD, OTHER } frame_type;

typedef enum { false, true } bool;

typedef enum { SEND_NUM_POINTS, SEND_POINTS, NUM_SEND_TYPES } send_flag;

typedef struct a_point{
	double x;
	double y;
	double z;
} point;

typedef struct point_frame {
	int num_points;
	point *points;
} frame;


point * get_more_space_and_copy(int *space_for, point *points, int so_far, frame_type type, int sizeof_item){
	
	if((*space_for) > so_far){
		return points;
	}
	int to_allocate = so_far > 0 ? so_far * 2 : (type == SKELETON ? 16 : 64);
	
	//	printf("mallocing from so_far %i to %i\n", so_far, to_allocate);
	point *temp = (point *) malloc (to_allocate * sizeof_item);
	
	if(points && so_far > 0){
		memcpy(temp, points, so_far * sizeof_item);
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
	frm->points = get_more_space_and_copy(&space_for_points, frm->points, frm->num_points, type, sizeof(point));
	point *current;
	
	while ((read = getline(&line, &len, handle)) != -1) {
		
		if(FRAME_DELIMITER == line[0]){
			printf("%i\n", frm->num_points);
			break;
		}else if(SAMPLE_PCL_POINTS && POINTCLOUD == type && rand() % 100 > PCL_POINTS_SAMPLE_RATE){
			continue;
		}
		
		
		frm->points = get_more_space_and_copy(&space_for_points, frm->points, frm->num_points, type, sizeof(point));
		current = &(frm->points[frm->num_points]);
		sscanf(line, "%lf,%lf,%lf", &(current->x), &(current->y), &(current->z));
		//		printf("READ LINE: %lf, %lf, %lf\n", current->x, current->y, current->z);
		
		frm->num_points++;
	}
	
	if(line){
		free(line);
	}
}

void send_frame_to(frame *frm, int destination){
	MPI_Send(&(frm->num_points), 1, MPI_INT, destination, SEND_NUM_POINTS, MPI_COMM_WORLD);
	MPI_Send(frm->points, frm->num_points * 3, MPI_DOUBLE, destination, SEND_POINTS, MPI_COMM_WORLD);
}

void get_frame_from(frame *frm, int source){
	MPI_Recv(&(frm->num_points), 1, MPI_INT, source, SEND_NUM_POINTS, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
	frm->points = (point *) malloc (frm->num_points * sizeof(point));
	MPI_Recv(frm->points, frm->num_points * 3, MPI_DOUBLE, source, SEND_POINTS, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
	printf("finished recv: %i\n", frm->num_points);
}

