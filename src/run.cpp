#include <time.h>
#include <signal.h>
#include <iostream>
#include <stdlib.h> 
#include <dlfcn.h>
#include <vector>
#include <stdio.h>
#include <unistd.h>

#include <sys/poll.h>

#include <pthread.h>

#include "util.h"
#include "poses.h"
#include "actions.h"
#include "cartesian_actions.h"
#include "viz.h"

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "Kinova.API.CommLayerUbuntu.h"
#include "KinovaTypes.h"

#define JOINT_DELTA_DEGREES 10
#define FINGER_DELTA_DEGREES 500

using namespace std;


// Borrowed heavily from the Kinova SDK examples

void print_help(){
	int i;
	
	cout << "General: " << endl;
	cout << "\thelp                           : show this text. " << endl;
	cout << "\tbegin                          : straighten arm, prepare for action. " << endl;
	cout << "\tstraighten                     : straighten arm. " << endl;
	cout << "\thome                           : put arm into home position. " << endl;
	cout << "\tshutdown                       : put arm into shutdown position. " << endl;
	cout << "\tquit                           : put arm into shutdown position then exit. " << endl;
	cout << "\tprint state                    : Print current arm/finger state. " << endl;
	cout << "\tr                              : Repeat previous command " << endl;

	cout << "Cartesian space: " << endl;
	cout << "\tcart home                      : send arm to home position using cartesian commands " << endl;
	cout << "\tcart goto x y z <tx ty tz>     : Kinect2 space: send arm to x, y, z, (with optional thetax, thetay, thetaz) using cartesian commands " << endl;

	cout << "Throwing: " << endl;
	cout << "\tload throw                     : put arm into loading position. " << endl;
	cout << "\tclose fingers                  : Close fingers for throw. " << endl;
	cout << "\topen fingers                   : Open fingers. " << endl;
	cout << "\tprep throw                     : Position arm for throw. " << endl;
	cout << "\tthrow                          : Throw. " << endl;

	cout << "Handoff: " << endl;
	cout << "\tgoto object                    : Move the JACO arm to the object. " << endl;
	cout << "\tgrab bottle                    : Close two fingers around bottle. " << endl;
	cout << "\treturn bottle                  : Go home, then return bottle to original grabbed location. " << endl;
	cout << "\trelease fingers                : Release all fingers. " << endl;
	cout << "\tUseful: home | go | gb | home | rb | home " << endl;


	cout << "Experiment: " << endl;
	cout << "\trun experiment                 : Begin user-study experiment, effectively running \"home | go | gb | home | rb | home\" " << endl;

	cout << "Specific motion: " << endl;
	cout << "\tmv <id> <angle>                : Move joint/finger to angle. IDs 0-5 are joints from base up, 6-8 are fingers." << endl;
	cout << "\tmv <id> <+/->                  : Increase/decrease a given actuator's angle (joints by " << JOINT_DELTA_DEGREES << " degrees, fingers by " << FINGER_DELTA_DEGREES << endl;
	cout << "Use | to chain commands \"load_throw | wait | prep throw | throw\"" << endl;
	cout << "\twait                           : Wait " << COMMAND_DELAY << " seconds" << endl;

	cout << "Visualization: " << endl;
	cout << "\tbottle color                   : set the bottle color to the output of 'grabc'. " << endl;
	cout << "\tpixel color                    : run 'grabc' to get a pixel color on-screen. " << endl;
	cout << "\tv <hd|qhd|sd>                     : change kinect ROS topic. " << endl;
	cout << "\tfind arm                       : toggle JACO arm finding. " << endl;
	cout << "\tv depth                        : toggle depth filter. " << endl;
	cout << "\tv table                        : toggle table removal. " << endl;
	cout << "\tv pixels                       : toggle pixel match color filter. " << endl;
	cout << "\tv verbose                      : toggle verbosity. " << endl;
	cout << "\tv frames <n>                   : Combine n past frames to smoooth pixel detection (default = " << DEFAULT_ADDITIONAL_COLOR_MATCH_FRAMES_TO_COMBINE << "). " << endl;
	cout << "\tv dist <+/->                   : Increase or decrease max recognition window (default = " << DEFAULT_MAX_INTERESTED_DISTANCE << "). " << endl;
	cout << "\txyxyz x,y                      : (KINECT) Map a 2-d point in the RGB to a 3-d coordinate." << endl;
	cout << "\tv cluster <";
	for(i = 0; i < NUMBER_OF_DETECTION_ALGORITHMS; i++){
		cout << detection_algorithm_names[i];
		if(i < NUMBER_OF_DETECTION_ALGORITHMS-1){
			cout << ",";
		}
	}
	cout << ">\n\t\t\tChange object detection algorithm (default = " << detection_algorithm_names[DEFAULT_OBJECT_DETECTION_ALG] << "). " << endl;
	cout << "ROS: " << endl;
	cout << "\tros wait                       : Begin waiting for ROS-input via " << INPUT_TOPIC << endl;
	cout << "\t(in ROS) rostopic pub " << INPUT_TOPIC << " --once std_msgs/String \"" << GO_HOME << ",\"        : Send arm home" << endl;
	
	
}


void handle_cartesian_goto(struct thread_args *args, char *cmd){
	MyMoveHome();
	
	string delim = " ";
	char * save_ptr;
	char * x;
	char * y;
	char * z;
	char * theta_x;
	char * theta_y;
	char * theta_z;
	
	x = strtok_r(cmd, delim.c_str(), &save_ptr);
	y = strtok_r(NULL, delim.c_str(), &save_ptr);
	z = strtok_r(NULL, delim.c_str(), &save_ptr);
	
	args->xyz_thetas.x = atof(x);
	args->xyz_thetas.y = atof(y);
	args->xyz_thetas.z = atof(z);
	
	theta_x = strtok_r(NULL, delim.c_str(), &save_ptr);
	if(theta_x){
		theta_y = strtok_r(NULL, delim.c_str(), &save_ptr);
		theta_z = strtok_r(NULL, delim.c_str(), &save_ptr);

	
		args->xyz_thetas.theta_x = atof(theta_x);
		args->xyz_thetas.theta_y = atof(theta_y);
		args->xyz_thetas.theta_z = atof(theta_z);
	}else{
		cout << "skipping goto thetas" << endl;
	}

	cout << args->xyz_thetas.x << "," << args->xyz_thetas.y << "," << args->xyz_thetas.z << "   theta: " << args->xyz_thetas.theta_x << "," << args->xyz_thetas.theta_y << "," << args->xyz_thetas.theta_z << endl;
	
	do_cartesian_action(args, true);
}


void handle_move_command(struct thread_args *args, char *cmd){
	string delim = " ";
	char * save_ptr;
	char * id;	
	char * angle;

	// strip mv	
	strtok_r(cmd, delim.c_str(), &save_ptr);
	
	id = strtok_r(NULL, delim.c_str(), &save_ptr);
	angle = id ? strtok_r(NULL, delim.c_str(), &save_ptr) : NULL;
	
	if(!id || !angle || 0 == strlen(angle)){
		cout << "Problem handling command." << endl;
		return;
	}
	
	int int_id = atoi(id);
	double double_angle;
	double delta;
	bool rotate_forward = '+' == angle[0];
	bool rotate_backward = '-' == angle[0];
	
	if(1 == strlen(id) && (rotate_forward || rotate_backward)){
		load_current_angles(args->angles);
		delta = (int_id < NUM_ACTUATORS) ? JOINT_DELTA_DEGREES : FINGER_DELTA_DEGREES;
		delta *= rotate_backward ? -1 : 1;
		args->angles[int_id] += delta;
	}else{
		double_angle = atof(angle);
		if(int_id < 0 || int_id >= NUM_COMPONENTS){
			cout << "Invalid joint/finger id. (0," << NUM_COMPONENTS << ")" << endl;
			return;
		}
		load_current_angles(args->angles);
		args->angles[int_id] = double_angle;
	}
	cout << "Moving " << int_id << " to " << args->angles[int_id] << endl;
	do_action(args, true);
}

void handle_detection_algorithm(struct viz_thread_args *viz_args, char * algorithm){
	int i;
	for(i = 0; i < NUMBER_OF_DETECTION_ALGORITHMS; i++){
		if(strcmp(algorithm, detection_algorithm_names[i]) == 0){
			viz_args->detection_algorithm = (object_detection_algorithm) i;
			break;
		}
	}
	
	cout << "Algorithm now: " << detection_algorithm_names[viz_args->detection_algorithm] << endl;
}

void handle_viz_distance(struct viz_thread_args *viz_args, char * num){
	if(num[0] == '+'){
		viz_args->max_interested_distance += MAX_INTERESTED_DISTANCE_INTERVAL;
	}else if(num[0] == '-'){
		viz_args->max_interested_distance -= MAX_INTERESTED_DISTANCE_INTERVAL;
	}
	cout << "Max distance: " << viz_args->max_interested_distance << endl;
}


void handle_viz_frames_to_combine(struct viz_thread_args *viz_args, char * num){
	viz_args->additional_color_match_frames_to_combine = atoi(num);
}

void normalize_xy(double x, double y, double *norm_x, double *norm_y){
	double v1[2] = {x,y};
	double length = vector_length_2d(v1);
	(*norm_x) = x / length;
	(*norm_y) = y / length;
}

void jaco_face_object(struct thread_args *args, struct cartesian_xyz *object_jaco_space){
	cout << "facing object" << endl;
	//cout << "x,y: " << object_jaco_space->x << "," << object_jaco_space->y << endl;
	// find rotation around a vector to (1,0), x axis 
	double x1 = object_jaco_space->x;
	double y1 = object_jaco_space->y;

	double x2 = -1;
	double y2 = 0;
	
	double degrees = degrees_between_2d_vectors(x1,y1,x2,y2);
	
	degrees = (double) (((int)degrees) % 360);
	cout << " degrees: " << degrees << endl;
	
	
	//cout << "radians: " << radians << " degrees: " << degrees << endl;
	
	load_current_angles(args->angles);
	args->angles[0] = degrees;
	do_action(args, true);
}



void goto_jaco_xyz(struct thread_args *args, struct viz_thread_args *viz_args, struct xyz *goal_xyz){
	
	int i;
	struct xyz *jaco_xyz = &(viz_args->jaco_tag_xyz[0]);
	
	struct cartesian_xyz object_in_jaco_space;

	// get object in jaco space
	translate_kinect_to_jaco(&object_in_jaco_space, goal_xyz, jaco_xyz);
	
	// rotate jaco arm to face object, assumes MyMoveHome() is current state.
	jaco_face_object(args, &object_in_jaco_space);

	// get object in jaco space
	translate_kinect_to_jaco(&object_in_jaco_space, goal_xyz, jaco_xyz);
	
	// rotate jaco arm to face object, assumes MyMoveHome() is current state.
	jaco_face_object(args, &object_in_jaco_space);

	// get object in jaco space again
	translate_kinect_to_jaco(&(args->xyz_thetas), goal_xyz, jaco_xyz);	
	
	// Go to above the object, then lower down
	double final_z = args->xyz_thetas.z;
	double approach = 0.1;
	args->xyz_thetas.z += approach;
	cout << "moving to object" << endl;
	do_cartesian_action(args, true);
	
	args->xyz_thetas.z -= approach;
	do_cartesian_action(args, true);
}


void hover_over_xyz(struct thread_args *args, struct viz_thread_args *viz_args, struct xyz *hover_at){

	MyMoveHome();

	// hovers over xyz
	struct xyz new_hover;
	new_hover.x = hover_at->x;
	new_hover.y = hover_at->y + 0.1;
	new_hover.z = hover_at->z;

	close_fingers_entirely(args);

	goto_jaco_xyz(args, viz_args, &new_hover);
}

void goto_object(struct thread_args *args, struct viz_thread_args *viz_args){
	if(viz_args->num_jaco_tags < 1){
		cout << "no arms to move" << endl;
		return;
	}else if(viz_args->num_objects < 1){
		cout << "no object in scene" << endl;
		return;
	}
	struct xyz *object_xyz = &(viz_args->object_xyz[0]);
	MyMoveHome();
	full_finger_release(args);
	goto_jaco_xyz(args, viz_args, object_xyz);
}

void return_object(struct thread_args *args, struct viz_thread_args *viz_args){
	if(viz_args->num_jaco_tags < 1){
		cout << "no arms to move" << endl;
		return;
	}else if(!args->object_grasped){
		cout << "not carrying an object" << endl;
		return;
	}
	
	MyMoveHome();
	
	// rotate jaco arm to face final destination
	jaco_face_object(args, & args->original_object_jaco_space_xyz_thetas);
	
	memcpy(& args->xyz_thetas, & args->original_object_jaco_space_xyz_thetas, sizeof(struct cartesian_xyz));
	
	// Go to above the object, then lower down
	double final_z = args->xyz_thetas.z + 0.5 * HEIGHT_OFFSET_ADJUSTMENT;
	double approach = 0.1;
	args->xyz_thetas.z += approach;
	cout << "moving to object" << endl;
	do_cartesian_action(args, true);
	
	args->xyz_thetas.z -= approach;
	do_cartesian_action(args, true);

	full_finger_release(args);
}

void run_experiment(struct thread_args *args, struct viz_thread_args *viz_args){
	// home | go | gb | home | rb | home
	go_home(args);
	
	goto_object(args, viz_args);
	
	grab_bottle(args);
	
	//go_home(args);
	
	return_object(args, viz_args);
	
	go_home(args);
}

void ros_wait(struct thread_args *args, struct viz_thread_args *viz_args){
	if(!args){
		cout << "No arm attached, breaking ros loop" << endl;
		return;
	}
	go_home(args);
	
	cout << "Press any key to break ROS listener and return." << endl;

	struct pollfd fds;
	fds.fd = 0; // STDIN
	fds.events = POLLIN;

	// flag that we're ready to receive.	
	args->ros_input->completed = true;

	while(poll(&fds, 1, 0) == 0){
		if(args->ros_input->ready){
			cout << "ready :: " << args->ros_input->msg << endl;
			args->ros_input->ready = false;
		
			if(args->ros_input->msg == HOVER_OVER_XYZ){
				cout << "Moving" << endl;
				hover_over_xyz(args, viz_args, & args->ros_input->move_to);
				cout << "done moving" << endl;
			}else if(args->ros_input->msg == GO_HOME){
				cout << "Going home" << endl;
				go_home(args);
				cout << "done moving" << endl;
			}

			args->ros_input->completed = true;
		}
		usleep(1000);
	}
	// flag that we're ready to receive.	
	args->ros_input->completed = false;
}

bool do_xyxyz_mapping(char *str, struct viz_thread_args *viz_args, double *xyz){
	char * result = std::strtok(str,",");

	int x = result ? atoi(result) : -1;
	result = std::strtok(NULL, ",");
	int y = result ? atoi(result) : -1;
	if(x >= 0 && y >= 0){
		get_xyz_from_xyzrgb(x, y, viz_args->cloud, xyz);
		return true;
	}
	return false;
}

void perform_xyxyz_mapping(struct viz_thread_args *viz_args, char *cmd){
	cout << "checking " << cmd << endl;
	double xyz[3];
	bool success = do_xyxyz_mapping(cmd, viz_args, xyz);

	if(success){
		cout << xyz[0] << "," << xyz[1] << "," << xyz[2] << endl;
	}else{
		cout << "Invalid 2-d coordinate" << endl;
	}
}

bool handle_cmd(int num_threads, struct thread_args *args, struct viz_thread_args *viz_args, const char *cmd, grasped_object_type object){
	
	struct thread_args *first_arm = num_threads > 0 ? &(args[0]) : NULL;

	if(!strcmp("begin", cmd)){
		straighten(first_arm);

	}else if(!strcmp("quit", cmd)){
		return false;

	}else if(!strcmp("run experiment", cmd) || !strcmp("re", cmd)){
		run_experiment(first_arm, viz_args);

	}else if(!strcmp("home", cmd)){
		go_home(first_arm);

	}else if(!strcmp("load throw", cmd)){
		load_throw(first_arm, object);

	}else if(!strcmp("close fingers", cmd)){
		close_fingers(first_arm, object);

	}else if(!strcmp("open fingers", cmd)){
		open_fingers(first_arm, object);

	}else if(!strcmp("release fingers", cmd) || !strcmp("rf", cmd)){
		full_finger_release(first_arm);

	}else if(!strcmp("prep throw", cmd)){
		prep_throw(first_arm, object);

	}else if(!strcmp("throw", cmd)){
		do_throw(first_arm, object);

	}else if(!strcmp("wait", cmd)){
		cout << "Waiting " << COMMAND_DELAY << " seconds" << endl;
		sleep(COMMAND_DELAY);
	
	}else if(!strcmp("print state", cmd) || !strcmp("ps", cmd)){
		print_state(viz_args);
	
	}else if(!strcmp("shutdown", cmd)){
		shutdown(first_arm);
	
	}else if(!strcmp("straighten", cmd)){
		straighten(first_arm);

	}else if(!strcmp("goto object", cmd) || !strcmp("go", cmd)){
		goto_object(first_arm, viz_args);

	}else if(!strcmp("hover object", cmd) || !strcmp("hover", cmd)){
		// passing in object's xyz, but could be any kinect-xyz
		if(viz_args->num_objects == 0){
			cout << "No object in scene" << endl;
		}
		hover_over_xyz(first_arm, viz_args, viz_args->object_xyz);

	}else if(!strcmp("grab bottle", cmd) || !strcmp("gb", cmd)){
		grab_bottle(first_arm);

	}else if(!strcmp("return bottle", cmd) || !strcmp("rb", cmd)){
		return_object(first_arm, viz_args);

	}else if(!strcmp("ros wait", cmd) || !strcmp("ros", cmd)){
		cout << "waiting for ros input" << endl;
		ros_wait(first_arm, viz_args);

	}else if(!strcmp("cart home", cmd)){
		cartesian_home(first_arm);

	}else if(strlen(cmd) > 10 && strncmp(cmd, "cart goto ", 10) == 0){
		handle_cartesian_goto(first_arm, (char *) &(cmd[10]));

	}else if(!strcmp("v verbose", cmd)){
		viz_args->verbose = !viz_args->verbose;
		cout << "verbose: " << (viz_args->verbose ? "On" : "Off") << endl;

	}else if(!strcmp("v hd", cmd)){
		viz_args->kinect_topic = (char *) KINECT_HD_TOPIC;
		cout << "Kinect topic: " << viz_args->kinect_topic << endl;

	}else if(!strcmp("v qhd", cmd)){
		viz_args->kinect_topic = (char *) KINECT_QHD_TOPIC;
		cout << "Kinect topic: " << viz_args->kinect_topic << endl;

	}else if(!strcmp("v sd", cmd)){
		viz_args->kinect_topic = (char *) KINECT_SD_TOPIC;
		cout << "Kinect topic: " << viz_args->kinect_topic << endl;

	}else if(!strcmp("v depth", cmd)){
		viz_args->draw_depth_filter = !viz_args->draw_depth_filter;
		cout << "depth shading: " << (viz_args->draw_depth_filter ? "On" : "Off") << endl;

	}else if(!strcmp("v table", cmd)){
		viz_args->remove_table = !viz_args->remove_table;
		cout << "remove_table: " << (viz_args->remove_table ? "On" : "Off") << endl;

	}else if(strlen(cmd) == 8 && strncmp(cmd, "v dist ", 7) == 0){
		handle_viz_distance(viz_args, (char *) &(cmd[7]) );

	}else if(strlen(cmd) > 9 && strncmp(cmd, "v frames ", 9) == 0){
		handle_viz_frames_to_combine(viz_args, (char *) &(cmd[9]) );

	}else if(!strcmp("v pixels", cmd)){
		viz_args->draw_pixel_match_color = !viz_args->draw_pixel_match_color;
		cout << "pixel drawing: " << (viz_args->draw_pixel_match_color ? "On" : "Off") << endl;

	}else if(!strcmp("find arm", cmd)){
		viz_args->find_arm = !viz_args->find_arm;
		cout << "arm finding: " << (viz_args->find_arm ? "On" : "Off") << endl;

	}else if(!strcmp("pixel color", cmd) || !strcmp("pc", cmd)){
		cout << "Click a pixel on screen." << endl;
		struct rgb color;
		
		get_onscreen_color(&color);
	}else if(!strcmp("bottle color", cmd) || !strcmp("bc", cmd)){
		cout << "Click a pixel on screen to set item of interest color." << endl;
		viz_args->orange_bottle_colors.num_colors = 1;
		get_onscreen_color(& (viz_args->orange_bottle_colors.colors[0]));		

	}else if(!strcmp("v highlight", cmd)){
		viz_args->highlight_visible_area = !viz_args->highlight_visible_area;
		cout << "highlight: " << (viz_args->highlight_visible_area ? "On" : "Off") << endl;
	}else if(strlen(cmd) > 6 && strncmp(cmd, "xyxyz ", 6) == 0){
		cout << "Performing mapping, with epsilon due to Kinect errors" << endl;
		perform_xyxyz_mapping(viz_args, (char *) &(cmd[6]));
		
	}else if(strlen(cmd) > 10 && strncmp(cmd, "v cluster ", 10) == 0){
		handle_detection_algorithm(viz_args, (char *) &(cmd[10]) );
	
	}else if(strlen(cmd) > 2 && strncmp(cmd, "mv ", 3) == 0){
		handle_move_command(first_arm, (char *) cmd);
	
	}else{
		print_help();
	}
	return true;
}

void do_repl(int num_threads, struct thread_args *args, struct viz_thread_args *viz_args){
	string cmd;
	int cmd_size = 1024, i;
	char cmd_char[cmd_size];
	string prompt = ">>";
	bool active = true;
	char * token;
	bool is_first_command = true;
	

	int last_cmd_size = 1024;
	char last_cmd_char[last_cmd_size];
	
	string delim = "|";
	
	grasped_object_type object = ORANGE;
	char * save_ptr;
	print_help();
	while(active && keepRunning){
		is_first_command = true;
		cout << prompt << " ";
		
		active = getline(cin, cmd);
		if(!active){
			break;
		}
		memset(cmd_char, 0, cmd_size);
		memcpy(cmd_char, cmd.c_str(), cmd.length());
		
		if(strlen(last_cmd_char) > 0 && strlen(cmd_char) == 1 && 'r' == cmd_char[0]){
			cout << "Repeat" << endl;
			memcpy(cmd_char, last_cmd_char, strlen(last_cmd_char));
			cmd_char[strlen(last_cmd_char)] = 0;
		}else{
			memcpy(last_cmd_char, cmd_char, strlen(cmd_char));
			last_cmd_char[strlen(cmd_char)] = 0;
		}
		
		token = strtok_r(cmd_char, delim.c_str(), &save_ptr);
		while(token && active){
			token = trimwhitespace(token);
			cout << "CMD: " << token << endl;
			active = handle_cmd(num_threads, args, viz_args, token, object);
			token = strtok_r(NULL, delim.c_str(), &save_ptr);
			is_first_command = false;
		}
	}
	
	if(!keepRunning){
		cout << "Caught signal, stopping" << endl;
	}

	
	for(i = 0; i < num_threads; i++){
		shutdown(&args[i]);
		args[i].shutdown = true;
		args->num_triggers = 0;
		if(args->triggers){
			free(args->triggers);
			args->triggers = NULL;
		}
	}
}

void *run_thread(void *thread_args){
	struct thread_args *args = (struct thread_args *) thread_args;
	int result = (*MyInitAPI)();

	cout << "("<<args->id<<") "<< "Thread Initialization's result: " << result << endl;
	MySetActiveDevice(*args->device);
	cout << "("<<args->id<<") "<< "Setting device to: " << args->device << endl;
	cout << "("<<args->id<<") "<< "args @ : " << args << endl;
	MyInitFingers();
	while(!args->shutdown){
		if(args->wake_up){
			args->wake_up = false; // race condition here, woken up between activation and this being set.
			
			layered_move(args->angles, args->triggers, args->num_triggers);
			args->completed_move = true;
		}else if(args->wake_up_cartesian){
			args->wake_up_cartesian = false; // race condition here, woken up between activation and this being set.
			
			layered_cartesian_move(& args->xyz_thetas);
			args->completed_move = true;
		}
		
		usleep(10000);
	}
	result = (*MyCloseAPI)();
}

class RosInputSubscriber{
	ros::NodeHandle nh_;
	ros::Subscriber sub_;
	struct ros_input *args;
	pthread_mutex_t completed_mutex;
	pthread_cond_t completed_cond;
	
	public:
	RosInputSubscriber(){
		completed_mutex = PTHREAD_MUTEX_INITIALIZER;
		completed_cond = PTHREAD_COND_INITIALIZER;
		sub_ = nh_.subscribe (INPUT_TOPIC, 1, &RosInputSubscriber::parse_input, this);
	}

	~RosInputSubscriber(){
		cout << "Tearing down ros subscriber" << endl;
	}

	void set_args(struct ros_input *ros_input){
		args = ros_input;
	}

	
	void parse_input(const std_msgs::String::ConstPtr& msg){
		if(args->terminate){
			return;
		}
		
		//pthread_mutex_lock(&completed_mutex);

		//pthread_cond_wait(&completed_cond, &completed_mutex);
		
		// Throw away commands until we're ready and accepting.
		if(!args->completed){
			//pthread_mutex_unlock(&completed_mutex);
			return;
		}

		char * cstr = new char [ msg->data.length()+1 ];
		std::strcpy (cstr, msg->data.c_str());
		cout << "ROS HEARD: " << cstr << endl;

		char * msg_type = std::strtok(cstr,",");

		args->msg = (ros_input_message_type) atoi(msg_type);
		if(args->msg == HOVER_OVER_XYZ){
			// Kinect 3-d coordinate already
			args->move_to.x = atof(std::strtok(NULL,","));
			args->move_to.y = atof(std::strtok(NULL,","));
			args->move_to.z = atof(std::strtok(NULL,","));
			cout << "move to: " << args->move_to.x << "," << args->move_to.y << "," << args->move_to.z << endl;
		}else if(args->msg == HOVER_OVER_XY){
			// Kinect 2-d RGB image coordinate, need to do mapping
			cout << "Trying " << &(cstr[2]) << endl;
			do_xyxyz_mapping(&(cstr[2]), args->viz_args, (double *) &args->move_to);
			cout << "move to: " << args->move_to.x << "," << args->move_to.y << "," << args->move_to.z << endl;
		}else if(args->msg == GO_HOME){
			// nothing additional
		}
		args->completed = true;
		args->ready = true;
		
		//pthread_cond_signal(&completed_cond);
		//pthread_mutex_unlock(&completed_mutex);
	}
	
	
};

void *handle_ros_input(void *thread_args){
	struct ros_input *ros_input = (struct ros_input *) thread_args;

	RosInputSubscriber ros_sub;
	ros_sub.set_args(ros_input);
	ros::Rate r(10);
	
	while(!ros_input->terminate){
		ros::spinOnce();
		r.sleep();
	}
	
	cout << endl << "Terminating ros input" << endl;
}

int main(int argc, char **argv){
	signal(SIGINT, intHandler);

	map<string, string> empty_mappings;
	ros::init(empty_mappings, "ros_subscribers");
	
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	
	KinovaDevice list[MAX_KINOVA_DEVICE];
	int devicesCount = 0;

	int result;

	//We load the handle for the library's command layer.
	void * commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so",RTLD_NOW|RTLD_GLOBAL);

	//We load the functions from the library
	MyInitAPI = (int (*)()) dlsym(commandLayer_handle,"InitAPI");
	MyCloseAPI = (int (*)()) dlsym(commandLayer_handle,"CloseAPI");
	MyMoveHome = (int (*)()) dlsym(commandLayer_handle,"MoveHome");
	MyInitFingers = (int (*)()) dlsym(commandLayer_handle,"InitFingers");
	MyGetDevices = (int (*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) dlsym(commandLayer_handle,"GetDevices");
	MySetActiveDevice = (int (*)(KinovaDevice devices)) dlsym(commandLayer_handle,"SetActiveDevice");
	MySendBasicTrajectory = (int (*)(TrajectoryPoint)) dlsym(commandLayer_handle,"SendBasicTrajectory");
	MyGetAngularCommand = (int (*)(AngularPosition &)) dlsym(commandLayer_handle,"GetAngularCommand");
	MyGetAngularPosition = (int (*)(AngularPosition &)) dlsym(commandLayer_handle,"GetAngularPosition");
	MyGetCartesianCommand = (int (*)(CartesianPosition &)) dlsym(commandLayer_handle,"GetCartesianCommand");

	if((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MySendBasicTrajectory == NULL) ||
	   (MySendBasicTrajectory == NULL) || (MyMoveHome == NULL) || (MyInitFingers == NULL) || 
		(MyGetCartesianCommand == NULL)){
		cout << "* * *  E R R O R   D U R I N G   I N I T I A L I Z A T I O N  * * *" << endl;
	}else{
		cout << "I N I T I A L I Z A T I O N   C O M P L E T E D" << endl << endl;

		result = (*MyInitAPI)();

		cout << "Initialization's result :" << result << endl;

		devicesCount = MyGetDevices(list, result);
		cout << "Found " << devicesCount << " JACO arms" << endl;
	}

	srand(time(NULL));
	struct viz_thread_args viz_args;
	memset(&viz_args, 0, sizeof(struct viz_thread_args));
	viz_args.terminate = false;
	viz_args.additional_color_match_frames_to_combine = DEFAULT_ADDITIONAL_COLOR_MATCH_FRAMES_TO_COMBINE;
	viz_args.draw_pixel_match_color = true;
	viz_args.draw_depth_filter = true;
	viz_args.num_jaco_arms_in_scene = DEFAULT_NUM_JACO_ARMS_IN_SCENE;
	viz_args.num_objects_in_scene = DEFAULT_NUM_OBJECTS_IN_SCENE;
	viz_args.max_interested_distance = DEFAULT_MAX_INTERESTED_DISTANCE;
	viz_args.detection_algorithm = DEFAULT_OBJECT_DETECTION_ALG;
	viz_args.highlight_visible_area = DEFAULT_HIGHLIGHT_VISIBLE_AREA;
	viz_args.visible_angle = DEFAULT_VISIBLE_ANGLE;
	viz_args.cloud = &cloud;
	viz_args.find_arm = DEFAULT_FIND_ARM;
	viz_args.kinect_topic = (char *) DEFAULT_KINECT_TOPIC;
	viz_args.remove_table = true;

	// set default colors
	memcpy(&viz_args.orange_bottle_colors, &orange_bottle_cylinder, sizeof(struct rgb_set));

	pthread_t viz_thread;
	pthread_create(&viz_thread, NULL, handle_viz, (void *) &viz_args);
	

	struct ros_input ros_input;
	memset(&ros_input, 0, sizeof(struct ros_input));
	ros_input.completed = false;
	pthread_t ros_input_thread;
	ros_input.viz_args = &viz_args;
	pthread_create(&ros_input_thread, NULL, handle_ros_input, (void *) &ros_input);
	
	
	pthread_t *threads = devicesCount > 0 ? (pthread_t *) malloc (devicesCount * sizeof(pthread_t)) : NULL;
	struct thread_args *args = devicesCount > 0 ? (struct thread_args *) malloc (devicesCount * sizeof(struct thread_args)) : NULL;
	if(devicesCount > 0){
		memset(args, 0, devicesCount * sizeof(struct thread_args));
	}
	
	for(int i = 0; i < devicesCount; i++){
		cout << "Found a robot on the USB bus (" << list[i].SerialNumber << ")" << endl;
		
		args[i].id = i;
		args[i].device = &list[i];
		args[i].viz_args = &viz_args;
		args[i].ros_input = &ros_input;
		
		pthread_create(&threads[i], NULL, run_thread, (void *) &(args[i]));
	}
	
	do_repl(devicesCount, args, &viz_args);

	dlclose(commandLayer_handle);
	
	viz_args.terminate = true;
	
	// suboptimal exit, but can't figure out closing CV window on exit otherwise.
	exit(0);
	
	//pthread_exit(NULL);
	//return 0;
}

