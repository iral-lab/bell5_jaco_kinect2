#include <stdio.h>
#include <string.h>
#include <iostream>
#include <dlfcn.h>
#include <vector>
#include <unistd.h>
#include <math.h>
#include <exception>

#include "Kinova.API.CommLayerUbuntu.h"
#include "KinovaTypes.h"

using namespace std;

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

#define NUM_ACTUATORS 6
#define NUM_FINGERS 3
#define NUM_COMPONENTS NUM_ACTUATORS+NUM_FINGERS

typedef enum {FINGER, ACTUATOR} component_type;

typedef enum {UNDEF, ORANGE} grasped_object_type;

class invalid_exception: public exception{
	virtual const char* what() const throw(){
	return "Invalid argument";
	}
} invalid_exception;

double angle_of_actuator(int actuator, AngularPosition *data_position){
	if(actuator < 0 || actuator >= NUM_ACTUATORS){
		cout << "requesting invalid actuator inside angle_of_actuator(" << actuator << ")" << endl;
		throw invalid_exception;
	}
	
	switch(actuator){
		case 0:
			return data_position->Actuators.Actuator1;
			break;
		case 1:
			return data_position->Actuators.Actuator2;
			break;
		case 2:
			return data_position->Actuators.Actuator3;
			break;
		case 3:
			return data_position->Actuators.Actuator4;
			break;
		case 4:
			return data_position->Actuators.Actuator5;
			break;
		case 5:
			return data_position->Actuators.Actuator6;
			break;
		default:
			throw invalid_exception; 
	}
}


double angle_of_finger(int finger, AngularPosition *data_position){
	if(finger < 0 || finger >= NUM_FINGERS){
		throw invalid_exception;
	}
	
	switch(finger){
		case 0:
			return data_position->Fingers.Finger1;
			break;
		case 1:
			return data_position->Fingers.Finger2;
			break;
		case 2:
			return data_position->Fingers.Finger3;
			break;
		default:
			throw invalid_exception; 
	}
}


void load_current_angles(int *angles){
	int i;
		
	memset(angles, 0, NUM_COMPONENTS * sizeof(int));
	
	(*MyGetAngularPosition)(data_position);
	for(i = 0; i < NUM_ACTUATORS; i++){
		angles[i] = angle_of_actuator(i, &data_position);
	}
	for(i = 0; i < NUM_FINGERS; i++){
		angles[NUM_ACTUATORS + i] = angle_of_finger(i, &data_position);
	}
}

void reset_point_to_send(TrajectoryPoint *point_to_send){
	point_to_send->Position.Type = ANGULAR_VELOCITY;

	point_to_send->Position.Actuators.Actuator1 = 0;
	point_to_send->Position.Actuators.Actuator2 = 0;
	point_to_send->Position.Actuators.Actuator3 = 0;
	point_to_send->Position.Actuators.Actuator4 = 0;
	point_to_send->Position.Actuators.Actuator5 = 0;
	point_to_send->Position.Actuators.Actuator6 = 0;

	point_to_send->Position.Fingers.Finger1 = 0;
	point_to_send->Position.Fingers.Finger2 = 0;
	point_to_send->Position.Fingers.Finger3 = 0;
}

void set_actuator_movement(int actuator, TrajectoryPoint *point_to_send, float movement){
	if(actuator < 0 || actuator >= NUM_ACTUATORS){
		throw invalid_exception;
	}
	
	switch(actuator){
		case 0:
			point_to_send->Position.Actuators.Actuator1 = movement;
			break;
		case 1:
			point_to_send->Position.Actuators.Actuator2 = movement;
			break;
		case 2:
			point_to_send->Position.Actuators.Actuator3 = movement;
			break;
		case 3:
			point_to_send->Position.Actuators.Actuator4 = movement;
			break;
		case 4:
			point_to_send->Position.Actuators.Actuator5 = movement;
			break;
		case 5:
			point_to_send->Position.Actuators.Actuator6 = movement;
			break;
		default:
			throw invalid_exception; 
	}
}


void set_finger_movement(int finger, TrajectoryPoint *point_to_send, float movement){
	if(finger < 0 || finger >= NUM_FINGERS){
		throw invalid_exception;
	}
	
	switch(finger){
		case 0:
			point_to_send->Position.Fingers.Finger1 = movement;
			break;
		case 1:
			point_to_send->Position.Fingers.Finger2 = movement;
			break;
		case 2:
			point_to_send->Position.Fingers.Finger3 = movement;
			break;
		default:
			throw invalid_exception; 
	}
}

void move_arm_to(int *angles){
	
	TrajectoryPoint point_to_send;
	point_to_send.InitStruct();
	reset_point_to_send(&point_to_send);
	
	int desired_angle = 0;
	double diff, to_move;
	bool arrived = false;
	
	int num_items = NUM_ACTUATORS + NUM_FINGERS;
	
	// higher allowance for fingers
	int epsilon;
	int speed;
	int i, actuator_number, finger_number;

	bool finished[num_items];
	for(i = 0; i < num_items; i++){
		finished[i] = false;
	}

	while(!arrived){
		arrived = true;
		(*MyGetAngularPosition)(data_position);
		for(i = 0; i < num_items; i++){
			if(finished[i]){
				continue;
			}

			bool is_actuator = i < NUM_ACTUATORS;
			bool is_finger = !is_actuator;
			epsilon = is_actuator ? 2 : 100;
			speed = is_actuator ? 60 : 2000;
			actuator_number = i;
			finger_number = max(0, i - NUM_ACTUATORS);

			desired_angle = angles[i];
			if(is_actuator){
				diff = angle_of_actuator(actuator_number, &data_position) - desired_angle;
			}else if(is_finger){
				diff = angle_of_finger(finger_number, &data_position) - desired_angle;
			}
			to_move = 0;
			if(fabs(diff) > epsilon){
				
				if(fabs(diff) <= epsilon * 2){
					// slow down so as to avoid thrashing
					to_move = (diff > 0 ? -1 : 1) * epsilon;
				}else{
					to_move = (diff > 0 ? -1 : 1) * speed;
				}
				
				arrived = false;
			}
			
			if(!finished[i] && 0 == to_move){
				cout << (is_actuator ? "\tFinished actuator " : "\tFinished finger ") << i << endl;
				finished[i] = true;
			}else if(is_actuator){
				set_actuator_movement(actuator_number, &point_to_send, to_move);
			}else if(is_finger){
				set_finger_movement(finger_number, &point_to_send, to_move);
			}
		}
		
		MySendBasicTrajectory(point_to_send);
		usleep(10000);
	}
}




void straighten(){
	cout << "Straightening arm" << endl;
	int angles[NUM_COMPONENTS];
	load_current_angles(angles);
	angles[1] = 180;
	
	move_arm_to(angles);
	
	angles[0] = 90;
	angles[1] = 180;
	angles[2] = 180;
	angles[3] = 90;
	angles[4] = 0;
	angles[5] = 0;

	move_arm_to(angles);
}

void prep_throw(){
	cout << "Prepping throw" << endl;

	int angles[NUM_COMPONENTS];
	// don't forget finger locations
	load_current_angles(angles);

	angles[0] = 0;
	angles[1] = 180;
	angles[2] = 180;
	angles[3] = 90;
	angles[4] = 0;
	angles[5] = 0;
	
	move_arm_to(angles);

	angles[0] = 0;
	angles[1] = 230;
	angles[2] = 90;
	angles[3] = -90;
	angles[4] = 0;
	angles[5] = 0;
	
	move_arm_to(angles);
}

void open_fingers(grasped_object_type object){
	cout << "Opening fingers" << endl;
	
	int angles[NUM_COMPONENTS];
	load_current_angles(angles);
	
	if(ORANGE == object){
		angles[NUM_ACTUATORS + 0] = angles[NUM_ACTUATORS + 1] = angles[NUM_ACTUATORS + 2] = 3500;
	}else{
		angles[NUM_ACTUATORS + 0] = angles[NUM_ACTUATORS + 1] = angles[NUM_ACTUATORS + 2] = 0;
	}
	move_arm_to(angles);
}

void close_fingers(grasped_object_type object){
	cout << "Closing fingers" << endl;
	int angles[NUM_COMPONENTS];
	load_current_angles(angles);
	
	if(ORANGE == object){
		angles[NUM_ACTUATORS + 0] = angles[NUM_ACTUATORS + 1] = angles[NUM_ACTUATORS + 2] = 4400;
	}else{
		angles[NUM_ACTUATORS + 0] = angles[NUM_ACTUATORS + 1] = angles[NUM_ACTUATORS + 2] = 5000;
	}
	move_arm_to(angles);
}

void do_throw(grasped_object_type object){
	cout << "Throwing" << endl;
	
	int angles[NUM_COMPONENTS];
	// don't forget finger locations
	load_current_angles(angles);
	
	angles[1] = 90;
	angles[2] = 180;
	move_arm_to(angles);
}

void load_throw(grasped_object_type object){
	cout << "Loading throw" << endl;

	int angles[NUM_COMPONENTS];
	// don't forget finger locations
	load_current_angles(angles);
	
	angles[0] = 0;
	angles[1] = 180;
	angles[2] = 180;
	angles[3] = 90;

	move_arm_to(angles);
	
	angles[0] = 0;
	angles[1] = 90;
	angles[2] = 180;
	angles[3] = 180;
	angles[4] = -90;
	angles[5] = 0;
	
	move_arm_to(angles);
	
	open_fingers(object);
}






















 
