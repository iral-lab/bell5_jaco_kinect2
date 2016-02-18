#include "actions.h"
#include <iostream>
#include <dlfcn.h>
#include <vector>
#include <stdio.h>
#include <unistd.h>

#include "Kinova.API.CommLayerUbuntu.h"
#include "KinovaTypes.h"

using namespace std;

// Borrowed heavily from the Kinova SDK examples

void print_help(){
	cout << "General: " << endl;
	cout << "\thelp          : show this text. " << endl;
	cout << "\tbegin         : straighten arm, prepare for action. " << endl;
	cout << "\tquit          : exit, put arm into shutdown position. " << endl;
	cout << "\tprint state   : Print current arm/finger state. " << endl;
	cout << "Throwing: " << endl;
	cout << "\tload throw    : put arm into loading position. " << endl;
	cout << "\tclose fingers : Close fingers for throw. " << endl;
	cout << "\topen fingers  : Open fingers. " << endl;
	cout << "\tprep throw    : Position arm for throw. " << endl;
	cout << "\tthrow         : Throw. " << endl;
	
}

void do_repl(){
	string cmd;
	int cmd_size = 1024;
	char cmd_char[cmd_size];
	string prompt = ">>";
	bool active = true;
	
	grasped_object_type object = ORANGE;
	
	print_help();
	while(active){
		cout << prompt << " ";
		
		getline(cin, cmd);
		
		if(!strcmp("begin", cmd.c_str())){
			straighten();

		}else if(!strcmp("quit", cmd.c_str())){
			active = false;
			break;

		}else if(!strcmp("load throw", cmd.c_str())){
			load_throw(object);

		}else if(!strcmp("close fingers", cmd.c_str())){
			close_fingers(object);

		}else if(!strcmp("open fingers", cmd.c_str())){
			open_fingers(object);

		}else if(!strcmp("prep throw", cmd.c_str())){
			prep_throw(object);

		}else if(!strcmp("throw", cmd.c_str())){
			do_throw(object);
		
		}else if(!strcmp("print state", cmd.c_str())){
			print_state();
		
		}else{
			print_help();
		}
	}
	shutdown();
}


int main(){
	
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

