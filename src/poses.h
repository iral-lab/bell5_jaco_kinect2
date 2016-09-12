#ifndef _POSESH_
#define _POSESH_

#include <iostream>
#include "util.h"
#include "actions.h"

using namespace std;

void straighten(struct thread_args *args){
	cout << "Straightening arm" << endl;
	
	load_current_angles(args->angles);
	args->angles[1] = 180;
	
	do_action(args, true);
	
	args->angles[0] = 0;
	args->angles[1] = 180;
	args->angles[2] = 180;
	args->angles[3] = 90;
	args->angles[4] = 0;
	args->angles[5] = 0;

	do_action(args, true);
}

void home_and_rotate(struct thread_args *args){
	MyMoveHome();
	return;
	cout << "Rotating" << endl;
	load_current_angles(args->angles);
	args->angles[0] = -188;
	do_action(args, true);
}

void go_home(struct thread_args *args){
	cout << "Going home" << endl;
	
	//straighten(args);
	args->arm_has_moved = true;
	
	home_and_rotate(args);
}

void open_fingers(struct thread_args *args, grasped_object_type object){
	cout << "Opening fingers" << endl;
	
	//int angles[NUM_COMPONENTS];
	load_current_angles(args->angles);
	
	if(ORANGE == object){
		args->angles[NUM_ACTUATORS + 2] = 6000;
		do_action(args, true);
		args->angles[NUM_ACTUATORS + 0] = args->angles[NUM_ACTUATORS + 1] = 3500;
		args->angles[NUM_ACTUATORS + 2] = 6000;
	}else{
		args->angles[NUM_ACTUATORS + 0] = args->angles[NUM_ACTUATORS + 1] = args->angles[NUM_ACTUATORS + 2] = 0;
	}
	do_action(args, true);
}

void full_finger_release(struct thread_args *args){
	MyInitFingers();
	cout << "Releasing fingers" << endl;
	
	load_current_angles(args->angles);
	
	args->angles[NUM_ACTUATORS + 0] = args->angles[NUM_ACTUATORS + 1] = args->angles[NUM_ACTUATORS + 2] = 0;
	do_action(args, true);
	args->object_grasped = false;
}

bool do_grab_bottle(struct thread_args *args, int close_to){
	cout << "Closing fingers" << endl;
	
	int epsilon = 500;
	
	MyInitFingers();
	
	load_current_angles(args->angles);
	
	// give the solo finger a head start
	args->angles[NUM_ACTUATORS + 0] = 2000;
	do_action(args, true);
	args->angles[NUM_ACTUATORS + 0] = args->angles[NUM_ACTUATORS + 1] = args->angles[NUM_ACTUATORS + 2] = close_to;
	do_action(args, true);
	
	load_current_angles(args->angles);
	int max_close = 7000;
	int distance_between_primary_fingers = (max_close - args->angles[NUM_ACTUATORS + 0]) + (max_close - args->angles[NUM_ACTUATORS + 1]) + (max_close - args->angles[NUM_ACTUATORS + 2]);
	int primary_fingers_should_be = (max_close - close_to) * 3;
	bool grasped_object = distance_between_primary_fingers > primary_fingers_should_be;
	
	cout << "distance: " << distance_between_primary_fingers << ", should be " << primary_fingers_should_be << " therefore " << (grasped_object ? "got it" : "missed") << endl;
	return grasped_object;
}

void grab_bottle(struct thread_args *args){
	
	int max_tries = 2;
	
	int close_to = 7000;
	
	int current_angles[NUM_COMPONENTS];
	bool success = false;
	while(max_tries > 0 && !success){
		success = do_grab_bottle(args, close_to);
		max_tries--;
		cout << "Close success: " << (success ? "yes" : "no") << ", tries left: " << max_tries << endl;
		if(!success){
			full_finger_release(args);
		}
	}
	if(success){
		args->object_grasped = true;
		load_current_cartesian_position(& args->original_object_jaco_space_xyz_thetas);
		cout << "Grasped object, remembering location" << endl;
	}
	
}


void close_fingers(struct thread_args *args, grasped_object_type object){
	MyInitFingers();
	cout << "Closing fingers" << endl;
	
	load_current_angles(args->angles);
	
	if(ORANGE == object){
		args->angles[NUM_ACTUATORS + 2] = 6000;
		args->angles[NUM_ACTUATORS + 0] = args->angles[NUM_ACTUATORS + 1] = 4400;
		do_action(args, true);
	}else{
		args->angles[NUM_ACTUATORS + 0] = args->angles[NUM_ACTUATORS + 1] = args->angles[NUM_ACTUATORS + 2] = 5000;
		do_action(args, true);
	}

}

void close_fingers_entirely(struct thread_args *args){
	MyInitFingers();
	cout << "Closing fingers all the way" << endl;
	
	load_current_angles(args->angles);
	
	args->angles[NUM_ACTUATORS + 0] = args->angles[NUM_ACTUATORS + 1] = args->angles[NUM_ACTUATORS + 2] = 6000;
	do_action(args, true);
}

void prep_throw(struct thread_args *args, grasped_object_type object){
	cout << "Prepping throw" << endl;
	close_fingers(args, object);
	
	load_current_angles(args->angles);

	args->angles[0] = -20; //120;
	
	args->angles[1] = 180;
	args->angles[2] = 180;
	args->angles[3] = -90;
	args->angles[4] = 0;
	args->angles[5] = 90;
	
	do_action(args, true);

	args->angles[1] = 230;
	args->angles[2] = 90;
	args->angles[3] = -90;
	args->angles[4] = 0;
	args->angles[5] = 90;
	
	do_action(args, true);
}

void elbow_first(struct thread_args *args, grasped_object_type object){
	args->angles[2] = 180;
	args->angles[3] = -90;
	
	args->num_triggers = 3;
	args->triggers = (struct actuator_trigger *) malloc (args->num_triggers * sizeof(struct actuator_trigger));
	memset(args->triggers, 0, args->num_triggers * sizeof(struct actuator_trigger));
	int so_far = 0;

	// move elbow before shoulder
	args->triggers[so_far].move_actuator = true;
	args->triggers[so_far].actuator_number = 2;
	args->triggers[so_far].actuator_position = 120; // lower is earlier
	args->triggers[so_far].new_actuator_number = 1;
	args->triggers[so_far].new_actuator_position = 90;
	so_far++;
	

	// release fingers
	args->triggers[so_far].move_finger = true;
	args->triggers[so_far].actuator_number = 2;
	args->triggers[so_far].actuator_position = 130;
	args->triggers[so_far].new_finger_number = 0;
	args->triggers[so_far].new_finger_position = 0;

	memcpy(&(args->triggers[so_far+1]), args->triggers, sizeof(struct actuator_trigger));
	
	args->triggers[so_far+1].new_finger_number = 1;
	so_far += 2;
	
	do_action(args, true);
}


void do_throw(struct thread_args *args, grasped_object_type object){
	cout << "Throwing" << endl;
	
	load_current_angles(args->angles);
	
	elbow_first(args, object);
}

void load_throw(struct thread_args *args, grasped_object_type object){
	cout << "Loading throw " << endl;
	clear_triggers(args);
	
	args->angles[0] = 0;
	args->angles[1] = 90;
	args->angles[2] = 180;
	args->angles[3] = 190;
	args->angles[4] = -60;
	args->angles[5] = 0;

	args->angles[6] = args->angles[7] = args->angles[8] = 3500;
	
	do_action(args, true);
	do_action(args, true);	
	open_fingers(args, object);
	
}


void shutdown(struct thread_args *args){
	if(!args->arm_has_moved){
		cout << "Arm never moved" << endl;
		return;
	}
	cout << "Going to shutdown position" << endl;
	
	straighten(args);
	
	load_current_angles(args->angles);
	
	args->angles[0] = -90;
	args->angles[1] = 160;
	args->angles[2] = 30;
	args->angles[3] = 270;
	args->angles[4] = 0;
	args->angles[5] = 0;
	args->angles[6] = 0;
	args->angles[7] = 0;
	args->angles[8] = 0;
	
	do_action(args, true);
	//close_fingers(args, UNDEF);
	
	load_current_angles(args->angles);
	args->angles[2] = 25;
	do_action(args, true);
	args->arm_has_moved = false;
}



#endif

