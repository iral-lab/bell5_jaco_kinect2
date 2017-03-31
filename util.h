typedef enum { SKELETON, POINTCLOUD, PATHS, OTHER } frame_type;

typedef enum { false, true } bool;

typedef enum { SEND_NUM_POINTS, SEND_POINTS, NUM_SEND_TYPES } send_flag;

// stored in terms of millimeters
typedef struct a_point{
	short x;
	short y;
	short z;
	int pid; // used for maintaining references between points
} point;

typedef struct point_frame {
	int num_points;
	point *points;
} frame;

void debug(char * str){
	printf("%s\n",str);
	fflush(stdout);
}

void randomize_array(void *list, int n, int size){
	if(n < 2){
		return;
	}
	void *temp = (void *) malloc(size);
	int random;
	int i;
	for(i = 0; i < n; i++){
		random = rand() % n;
		memcpy(temp, &(list[i * size]), size);
		memcpy(&(list[i * size]), &(list[random * size]), size);
		memcpy(&(list[random * size]), temp, size);
	}
}

void * get_more_space_and_copy(int *space_for, void *some_list, int so_far, frame_type type, int sizeof_item){
	
	if((*space_for) > so_far){
		return some_list;
	}
	int to_allocate = so_far > 0 ? so_far * 2 : (type == SKELETON ? 16 : (type == PATHS ? 128 : 64));
	
//	printf("mallocing from so_far %i to %i\n", so_far, to_allocate);
	void *temp = (point *) malloc (to_allocate * sizeof_item);
	memset(temp, 0, to_allocate * sizeof_item);
	
	if(some_list && so_far > 0){
		memcpy(temp, some_list, so_far * sizeof_item);
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
	
	double temp_x, temp_y, temp_z;
	while ((read = getline(&line, &len, handle)) != -1) {
		
		if(FRAME_DELIMITER == line[0]){
//			printf("%i\n", frm->num_points);
			break;
		}else if(SAMPLE_PCL_POINTS && POINTCLOUD == type && rand() % 100 > PCL_POINTS_SAMPLE_RATE){
			continue;
		}
		
		
		frm->points = get_more_space_and_copy(&space_for_points, frm->points, frm->num_points, type, sizeof(point));
		current = &(frm->points[frm->num_points]);
		sscanf(line, "%lf,%lf,%lf", &temp_x, &temp_y, &temp_z);
		current->x = floorf(temp_x * UNIT_SCALAR);
		current->y = floorf(temp_y * UNIT_SCALAR);
		current->z = floorf(temp_z * UNIT_SCALAR);
		
//		printf("READ LINE: %i, %i, %i\n", current->x, current->y, current->z);
		
		frm->num_points++;
	}
//	printf("num points: %i\n", frm->num_points);
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
	int i;
	for(i = 0; i < num_frames; i++){
		point_count += frames[i].num_points;
	}
	//	printf("total points: %i\n", point_count);
	
	MPI_Bcast(&point_count, 1, MPI_INT, 0, MPI_COMM_WORLD);
	//	printf("copied the fact that its %i points\n", point_count);
	
	
	// pack points and send
	int *points_per_frame = (int *) malloc (point_count * sizeof(int));
	point *points = (point *) malloc (point_count * sizeof(point));
	int point_i = 0;
	for(i = 0; i < num_frames; i++){
		memcpy(&(points[point_i]), frames[i].points, frames[i].num_points * sizeof(point));
		
		points_per_frame[i] = frames[i].num_points;
		point_i += frames[i].num_points;
	}

	MPI_Bcast(points, point_count * sizeof(point), MPI_BYTE, 0, MPI_COMM_WORLD);
//	printf("packed/sent %i frame points, %i bytes\n", point_count, point_count * sizeof(point));
	
	// send number of points per frame
	MPI_Bcast(points_per_frame, point_count, MPI_INT, 0, MPI_COMM_WORLD);
	
}

void receive_and_unpack(int rank, int *num_frames, frame **frames, point **points){
	// get number of frames
	MPI_Bcast(num_frames, 1, MPI_INT, 0, MPI_COMM_WORLD);
	(*frames) = (frame *) malloc ((*num_frames) * sizeof(frame));
	memset(*frames, 0, (*num_frames) * sizeof(frame));
	
	// get total number of packed points, make room
	int total_points;
	MPI_Bcast(&total_points, 1, MPI_INT, 0, MPI_COMM_WORLD);
	(*points) = (point *) malloc (total_points * sizeof(point));
	memset(*points, 0, total_points * sizeof(point));
	
	// get packed points
	MPI_Bcast(*points, total_points * sizeof(point), MPI_BYTE, 0, MPI_COMM_WORLD);
	
	// get counts, break up frame points
	int *points_per_frame = (int *) malloc (total_points * sizeof(int));
	MPI_Bcast(points_per_frame, total_points, MPI_INT, 0, MPI_COMM_WORLD);
	
	// build frames
	int point_i = 0;
	int i;
	for(i = 0; i < (*num_frames); i++){
		
		(*frames)[i].num_points = points_per_frame[i];
		(*frames)[i].points = &((*points)[point_i]);
		
		point_i += points_per_frame[i];
	}
	
}

void validate_frames(int rank, int num_frames, frame *frames){
	int i,j,k;
	for(k = 0; k < num_frames; k++){
		frame *frm = &(frames[k]);
		for(j = 0; j < frm->num_points; j++){
			if(frm->points[j].x == 0 || frm->points[j].y == 0 || frm->points[j].z == 0){
				printf("%i HAS INVALID FRAME %i\n", rank, j);
				printf("Frame points %i\n", frm->num_points);
				for(i = 0; i < frm->num_points; i++){
					printf("%i %i %i\n", frm->points[i].x, frm->points[i].y,frm->points[i].z);
				}
				exit(1);
			}
		}
	}
	
}


void read_and_broadcast_frames(char * input_skeleton_file, char * input_pcl_file, int *num_skeleton_frames, frame **all_skeleton_frames, int *num_pointcloud_frames, frame **all_pointcloud_frames){
	
	FILE *skeleton_handle = fopen(input_skeleton_file, "r");
	FILE *pcl_handle = fopen(input_pcl_file, "r");
	
	if(!skeleton_handle){
		printf("Skeleton file does not exist: %s\n", input_skeleton_file);
		exit(1);
	}else if(!pcl_handle){
		printf("PCL file does not exist: %s\n", input_pcl_file);
		exit(1);
	}
	
	read_frames(skeleton_handle, SKELETON, num_skeleton_frames, all_skeleton_frames);
	printf("done reading in %i skeleton frames\n", *num_skeleton_frames);
	
	int i;
	for(i = 0; i < (*num_skeleton_frames); i++){
		if((*all_skeleton_frames)[i].num_points > MAX_POINTS_IN_SKELETON_FRAME){
			printf("Too many points in a skeleton frame %i (planned for max %i)\n", (*all_skeleton_frames)[i].num_points, MAX_POINTS_IN_SKELETON_FRAME);
			exit(1);
		}
	}
	
	read_frames(pcl_handle, POINTCLOUD, num_pointcloud_frames, all_pointcloud_frames);
	printf("done reading in %i pointcloud frames\n", *num_pointcloud_frames);
	
	printf("0 validating skeleton\n");
	validate_frames(0, *num_skeleton_frames, *all_skeleton_frames);
	
	printf("0 validating pcl\n");
	validate_frames(0, *num_pointcloud_frames, *all_pointcloud_frames);
	
	
	MPI_Bcast(num_skeleton_frames, 1, MPI_INT, 0, MPI_COMM_WORLD);
	MPI_Bcast(num_pointcloud_frames, 1, MPI_INT, 0, MPI_COMM_WORLD);
	
	pack_and_send(*num_skeleton_frames, *all_skeleton_frames);
	pack_and_send(*num_pointcloud_frames, *all_pointcloud_frames);
	
	if(skeleton_handle){
		fclose(skeleton_handle);
	}
	if(pcl_handle){
		fclose(pcl_handle);
	}
}

void listen_for_frames(int rank, int *num_skeleton_frames, frame **all_skeleton_frames, point **skeleton_packed_points, int *num_pointcloud_frames, frame **all_pointcloud_frames, point **pointcloud_packed_points){
	MPI_Bcast(num_skeleton_frames, 1, MPI_INT, 0, MPI_COMM_WORLD);
	MPI_Bcast(num_pointcloud_frames, 1, MPI_INT, 0, MPI_COMM_WORLD);
//		printf("%i > %i skeleton frames, %i pointcloud frames\n", rank, *num_skeleton_frames, *num_pointcloud_frames);
	
	receive_and_unpack(rank, num_skeleton_frames, all_skeleton_frames, skeleton_packed_points);
//	printf("> %i Finished rebuilding %i skeleton frames\n", rank, *num_skeleton_frames);
	validate_frames(rank, *num_skeleton_frames, *all_skeleton_frames);
	
	
	receive_and_unpack(rank, num_pointcloud_frames, all_pointcloud_frames, pointcloud_packed_points);
//	printf("> %i Finished rebuilding %i pointcloud frames\n", rank, *num_pointcloud_frames);
	validate_frames(rank, *num_pointcloud_frames, *all_pointcloud_frames);
	
}



