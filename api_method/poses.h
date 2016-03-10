#ifndef _POSESH_
#define _POSESH_

#include "actions.h"


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

void close_fingers(struct thread_args *args, grasped_object_type object){
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
	cout << "Loading throw" << endl;
	clear_triggers(args);
	
	// don't forget finger locations
	load_current_angles(args->angles);
	
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
	close_fingers(args, UNDEF);
	
	load_current_angles(args->angles);
	args->angles[2] = 25;
	do_action(args, true);
}



#endif

