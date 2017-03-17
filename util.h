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

void debug(char * str){
	printf("%s\n",str);
	fflush(stdout);
}


void * get_more_space_and_copy(int *space_for, void *some_list, int so_far, frame_type type, int sizeof_item){
	
	if((*space_for) > so_far){
		return some_list;
	}
	int to_allocate = so_far > 0 ? so_far * 2 : (type == SKELETON ? 16 : 64);
	
	//	printf("mallocing from so_far %i to %i\n", so_far, to_allocate);
	void *temp = (point *) malloc (to_allocate * sizeof_item);
	
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
	
	while ((read = getline(&line, &len, handle)) != -1) {
		
		if(FRAME_DELIMITER == line[0]){
//			printf("%i\n", frm->num_points);
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
