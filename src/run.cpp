#include <time.h>
#include <signal.h>
#include <iostream>
#include <stdlib.h> 
#include <dlfcn.h>
#include <vector>
#include <stdio.h>
#include <unistd.h>

#include <pthread.h>

#include "util.h"
#include "actions.h"
#include "poses.h"
#include "viz.h"

#include "Kinova.API.CommLayerUbuntu.h"
#include "KinovaTypes.h"

#define COMMAND_DELAY 3

#define JOINT_DELTA_DEGREES 10
#define FINGER_DELTA_DEGREES 500

using namespace std;

// Borrowed heavily from the Kinova SDK examples

void print_help(){
	cout << "General: " << endl;
	cout << "\thelp            : show this text. " << endl;
	cout << "\tbegin           : straighten arm, prepare for action. " << endl;
	cout << "\tstraighten      : straighten arm. " << endl;
	cout << "\thome            : put arm into home position. " << endl;
	cout << "\tshutdown        : put arm into shutdown position. " << endl;
	cout << "\tquit            : put arm into shutdown position then exit. " << endl;
	cout << "\tprint state     : Print current arm/finger state. " << endl;
	cout << "\tr               : Repeat previous command " << endl;
	cout << "Throwing: " << endl;
	cout << "\tload throw      : put arm into loading position. " << endl;
	cout << "\tclose fingers   : Close fingers for throw. " << endl;
	cout << "\topen fingers    : Open fingers. " << endl;
	cout << "\tprep throw      : Position arm for throw. " << endl;
	cout << "\tthrow           : Throw. " << endl;
	cout << "Specific motion: " << endl;
	cout << "\tmv <id> <angle> : Move joint/finger to angle. IDs 0-5 are joints from base up, 6-8 are fingers." << endl;
	cout << "\tmv <id> <+/->   : Increase/decrease a given actuator's angle (joints by " << JOINT_DELTA_DEGREES << " degrees, fingers by " << FINGER_DELTA_DEGREES << endl;
	cout << "Use | to chain commands \"load_throw | wait | prep throw | throw\"" << endl;
	cout << "\twait            : Wait " << COMMAND_DELAY << " seconds" << endl;
	
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

bool handle_cmd(int num_threads, struct thread_args *args, const char *cmd, grasped_object_type object){
	
	if(!strcmp("begin", cmd)){
		straighten(&args[0]);

	}else if(!strcmp("quit", cmd)){
		return false;

	}else if(!strcmp("home", cmd)){
		go_home(&args[0]);

	}else if(!strcmp("load throw", cmd)){
		load_throw(&args[0], object);

	}else if(!strcmp("close fingers", cmd)){
		close_fingers(&args[0], object);

	}else if(!strcmp("open fingers", cmd)){
		open_fingers(&args[0], object);

	}else if(!strcmp("prep throw", cmd)){
		prep_throw(&args[0], object);

	}else if(!strcmp("throw", cmd)){
		do_throw(&args[0], object);

	}else if(!strcmp("wait", cmd)){
		cout << "Waiting " << COMMAND_DELAY << " seconds" << endl;
		sleep(COMMAND_DELAY);
	
	}else if(!strcmp("print state", cmd)){
		print_state();
	
	}else if(!strcmp("shutdown", cmd)){
		shutdown(&args[0]);
	
	}else if(!strcmp("straighten", cmd)){
		straighten(&args[0]);
	
	}else if(strlen(cmd) > 2 && cmd[0] == 'm' && cmd[1] == 'v'){
		handle_move_command(&args[0], (char *) cmd);
	
	}else{
		print_help();
	}
	return true;
}

void do_repl(int num_threads, struct thread_args *args){
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
		
		getline(cin, cmd);
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
			active = handle_cmd(num_threads, args, token, object);
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
	while(!args->shutdown){
		if(args->wake_up){
			args->wake_up = false; // race condition here, woken up between activation and this being set.
			
			layered_move(args->angles, args->triggers, args->num_triggers);
			args->completed_move = true;
		}
		
		usleep(10000);
	}
	result = (*MyCloseAPI)();
}


int main(int argc, char **argv){
	signal(SIGINT, intHandler);
	
	int result;
	srand(time(NULL));
	struct viz_thread_args viz_args;
	viz_args.argc = &argc;
	viz_args.argv = &argv;
	viz_args.terminate = false;
	
	pthread_t viz_thread;
	pthread_create(&viz_thread, NULL, handle_viz, (void *) &viz_args);
	
	
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

	if((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MySendBasicTrajectory == NULL) ||
	   (MySendBasicTrajectory == NULL) || (MyMoveHome == NULL) || (MyInitFingers == NULL))
	{
		cout << "* * *  E R R O R   D U R I N G   I N I T I A L I Z A T I O N  * * *" << endl;
	}
	else
	{
		cout << "I N I T I A L I Z A T I O N   C O M P L E T E D" << endl << endl;

		result = (*MyInitAPI)();

		cout << "Initialization's result :" << result << endl;

		KinovaDevice list[MAX_KINOVA_DEVICE];

		int devicesCount = MyGetDevices(list, result);

		pthread_t *threads = (pthread_t *) malloc (devicesCount * sizeof(pthread_t));
		struct thread_args *args = (struct thread_args *) malloc (devicesCount * sizeof(struct thread_args));
		memset(args, 0, devicesCount * sizeof(struct thread_args));
		
		for(int i = 0; i < devicesCount; i++){
			cout << "Found a robot on the USB bus (" << list[i].SerialNumber << ")" << endl;
			
			args[i].id = i;
			args[i].device = &list[i];
			
			pthread_create(&threads[i], NULL, run_thread, (void *) &(args[i]));
		}
		
		
		do_repl(devicesCount, args);
		

		//cout << endl << "WARNING: Your robot is now set to angular control. If you use the joystick, it will be a joint by joint movement." << endl;
		cout << endl << "C L O S I N G   A P I" << endl;
		// result = (*MyCloseAPI)();
	}

	pthread_join(viz_thread, NULL);
	
	dlclose(commandLayer_handle);
	pthread_exit(NULL);
	return 0;
}

