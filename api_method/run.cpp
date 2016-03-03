#include "actions.h"
#include <signal.h>
#include <iostream>
#include <stdlib.h> 
#include <dlfcn.h>
#include <vector>
#include <stdio.h>
#include <unistd.h>

#include "util.h"

#include "Kinova.API.CommLayerUbuntu.h"
#include "KinovaTypes.h"

#define COMMAND_DELAY 3

using namespace std;

// Borrowed heavily from the Kinova SDK examples

void print_help(){
	cout << "General: " << endl;
	cout << "\thelp            : show this text. " << endl;
	cout << "\tbegin           : straighten arm, prepare for action. " << endl;
	cout << "\tstraighten      : straighten arm. " << endl;
	cout << "\tshutdown        : put arm into shutdown position. " << endl;
	cout << "\tquit            : put arm into shutdown position then exit. " << endl;
	cout << "\tprint state     : Print current arm/finger state. " << endl;
	cout << "Throwing: " << endl;
	cout << "\tload throw      : put arm into loading position. " << endl;
	cout << "\tclose fingers   : Close fingers for throw. " << endl;
	cout << "\topen fingers    : Open fingers. " << endl;
	cout << "\tprep throw      : Position arm for throw. " << endl;
	cout << "\tthrow           : Throw. " << endl;
	cout << "Specific motion: " << endl;
	cout << "\tmv <id> <angle> : Move joint/finger to angle. IDs 0-5 are joints from base up, 6-8 are fingers." << endl;
	cout << "Use | to chain commands \"load_throw | wait | prep throw | throw\"" << endl;
	cout << "\twait            : Wait " << COMMAND_DELAY << " seconds" << endl;
	
}

void handle_move_command(char *cmd){
	string delim = " ";
	char * save_ptr;
	char * id;	
	char * angle;

	// strip mv	
	strtok_r(cmd, delim.c_str(), &save_ptr);
	
	id = strtok_r(NULL, delim.c_str(), &save_ptr);
	angle = id ? strtok_r(NULL, delim.c_str(), &save_ptr) : NULL;
	
	if(!id || !angle){
		cout << "Problem handling command." << endl;
		return;
	}
	int int_id = atoi(id);
	double double_angle = atof(angle);
	if(int_id < 0 || int_id >= NUM_COMPONENTS){
		cout << "Invalid joint/finger id. (0," << NUM_COMPONENTS << ")" << endl;
		return;
	} 
	cout << "Moving " << int_id << " to " << double_angle << endl;
	move_joint_to(int_id, double_angle);
}

bool handle_cmd(const char *cmd, grasped_object_type object){
	
	if(!strcmp("begin", cmd)){
		straighten();

	}else if(!strcmp("quit", cmd)){
		return false;

	}else if(!strcmp("load throw", cmd)){
		load_throw(object);

	}else if(!strcmp("close fingers", cmd)){
		close_fingers(object);

	}else if(!strcmp("open fingers", cmd)){
		open_fingers(object);

	}else if(!strcmp("prep throw", cmd)){
		prep_throw(object);

	}else if(!strcmp("throw", cmd)){
		do_throw(object);

	}else if(!strcmp("wait", cmd)){
		cout << "Waiting " << COMMAND_DELAY << " seconds" << endl;
		sleep(COMMAND_DELAY);
	
	}else if(!strcmp("print state", cmd)){
		print_state();
	
	}else if(!strcmp("shutdown", cmd)){
		shutdown();
	
	}else if(!strcmp("straighten", cmd)){
		straighten();
	
	}else if(strlen(cmd) > 2 && cmd[0] == 'm' && cmd[1] == 'v'){
		handle_move_command((char *) cmd);
	
	}else{
		print_help();
	}
	return true;
}

void do_repl(){
	string cmd;
	int cmd_size = 1024;
	char cmd_char[cmd_size];
	string prompt = ">>";
	bool active = true;
	char * token;
	bool is_first_command = true;
	
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
		
		token = strtok_r(cmd_char, delim.c_str(), &save_ptr);
		while(token && active){
			token = trimwhitespace(token);
			cout << "CMD: " << token << endl;
			active = handle_cmd(token, object);
			token = strtok_r(NULL, delim.c_str(), &save_ptr);
			is_first_command = false;
		}
	}

	if(!keepRunning){
		cout << "Caught signal, stopping" << endl;
	}else{
		shutdown();
	}

}


int main(){
	signal(SIGINT, intHandler);
	
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

		for(int i = 0; i < devicesCount; i++){
			cout << "Found a robot on the USB bus (" << list[i].SerialNumber << ")" << endl;

			//Setting the current device as the active device.
			MySetActiveDevice(list[i]);
		}
		
		
		do_repl();
		

		//cout << endl << "WARNING: Your robot is now set to angular control. If you use the joystick, it will be a joint by joint movement." << endl;
		cout << endl << "C L O S I N G   A P I" << endl;
		result = (*MyCloseAPI)();
	}

	dlclose(commandLayer_handle);

	return 0;
}

