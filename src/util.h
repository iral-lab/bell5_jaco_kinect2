#ifndef _UTILH_
#define _UTILH_

#include <string.h>

#include "Kinova.API.CommLayerUbuntu.h"
#include "KinovaTypes.h"


#define NUM_ACTUATORS 6
#define NUM_FINGERS 3
#define NUM_COMPONENTS NUM_ACTUATORS+NUM_FINGERS

AngularPosition current_command;
AngularPosition data_command;
AngularPosition data_position;

//Function pointers to the functions we need
int (*MyInitAPI)();
int (*MyCloseAPI)();
int (*MySendBasicTrajectory)(TrajectoryPoint command);
int (*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
int (*MySetActiveDevice)(KinovaDevice device);
int (*MyMoveHome)();
int (*MyInitFingers)();
int (*MyGetAngularCommand)(AngularPosition &);
int (*MyGetAngularPosition)(AngularPosition &);
int (*MyGetCartesianCommand)(CartesianPosition &);

struct xyz{
	double x;
	double y;
	double z;
};

struct viz_thread_args{
	int *argc;
	char ***argv;
	bool terminate;
	bool draw_depth_filter;
	bool draw_pixel_match_color;
	bool verbose;
	struct xyz *jaco_tag_xyz;
	double *jaco_distances;
	int num_jaco_tags;
	struct xyz *object_xyz;
	double *object_distances;
	int num_objects;
};


// trim functions from http://stackoverflow.com/questions/122616/how-do-i-trim-leading-trailing-whitespace-in-a-standard-way
char *trimwhitespace(char *str){
  char *end;

  // Trim leading space
  while(isspace(*str)) str++;

  if(*str == 0)  // All spaces?
    return str;

  // Trim trailing space
  end = str + strlen(str) - 1;
  while(end > str && isspace(*end)) end--;

  // Write new null terminator
  *(end+1) = 0;

  return str;
}

struct actuator_trigger{
	int actuator_number;
	float actuator_position;
	
	bool move_actuator;
	int new_actuator_number;
	float new_actuator_position;

	bool move_finger;
	int new_finger_number;
	float new_finger_position;
	
	bool triggered;
};

struct thread_args{
	int id;
	KinovaDevice *device;
	
	// Track what the goal angles are for each actuator/finger of the arm
	int angles[NUM_COMPONENTS];
	
	// Triggers that can be matched during move execution
	int num_triggers;
	struct actuator_trigger *triggers;

	// track if arm state is dirty, avoid straightening/shutdown if never moved.	
	bool arm_has_moved;
	
	// flag to wake the thread back up
	bool wake_up;
	
	// thread will set this to true when current instructions are met
	bool completed_move;
	
	// shutdown flag, terminate pthread
	bool shutdown;

	// pass info to the visualizer
	struct viz_thread_args *viz_args;
};

#endif

