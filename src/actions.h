#ifndef _ACTIONSH_
#define _ACTIONSH_


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <dlfcn.h>
#include <vector>
#include <unistd.h>
#include <math.h>
#include <exception>
#include <signal.h>

#include "util.h"

void straighten(struct thread_args *args);
void load_current_cartesian_position(struct cartesian_xyz *xyz_thetas);

static volatile int keepRunning = 1;
void intHandler(int dummy) {keepRunning = 0;}


using namespace std;

typedef enum {FINGER, ACTUATOR} component_type;

typedef enum {UNDEF, ORANGE, BOTTLE} grasped_object_type;

class invalid_exception: public exception{
	virtual const char* what() const throw(){
	return "Invalid argument";
	}
} invalid_exception;



void do_action(struct thread_args *args, bool blocking){
	if(!args->arm_has_moved){
		args->arm_has_moved = true;
		cout << "Moving home" << endl;
		// don't straighten here, overwrites what the prior move was
		MyMoveHome();
	}
	
	args->completed_move = false;
	args->wake_up = true;
	while(blocking && !args->completed_move){
		usleep(100);
	}
}

void clear_triggers(struct thread_args *args){
	args->num_triggers = 0;
	if(args->triggers){
		free(args->triggers);
		args->triggers = NULL;
	}
}

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

void layered_move(int *angles, struct actuator_trigger *triggers, int num_triggers){
	
	if(num_triggers > 0){
		for(int i = 0; i < num_triggers; i++){	
			cout << "handling trigger: A = " << triggers[i].actuator_number << " @ " << triggers[i].actuator_position << ", move finger " << triggers[i].new_finger_number << " to " << triggers[i].new_finger_position << endl;
		}
	}

	TrajectoryPoint point_to_send;
	point_to_send.InitStruct();
	reset_point_to_send(&point_to_send);
	
	int desired_angle = 0;
	double diff, to_move, current_angle;
	bool arrived = false;
	
	int num_items = NUM_ACTUATORS + NUM_FINGERS;
	
	// higher allowance for fingers
	int epsilon;
	int speed;
	int i, j, actuator_number, finger_number;

	bool finished[num_items];
	for(i = 0; i < num_items; i++){
		finished[i] = false;
	}
	int offset;
	
	int num_points_sent = 0;
	int max_ineffectual_tries = 10 * num_items;
	int tries_without_change = 0;
	double current_angles[num_items];
	memset(current_angles, 0, num_items * sizeof(double));
	
	while(!arrived && keepRunning && tries_without_change < max_ineffectual_tries){
		arrived = true;
		(*MyGetAngularPosition)(data_position);
		for(i = 0; i < num_items; i++){
			if(finished[i] || !keepRunning){
				continue;
			}

			bool is_actuator = i < NUM_ACTUATORS;
			bool is_finger = !is_actuator;
			epsilon = is_actuator ? 2 : 50;
			speed = is_actuator ? 60 : 2000;
			actuator_number = i;
			finger_number = max(0, i - NUM_ACTUATORS);

			desired_angle = angles[i];
			if(is_actuator){
				current_angle = angle_of_actuator(actuator_number, &data_position);
				diff = current_angle - desired_angle;
			}else if(is_finger){
				current_angle = angle_of_finger(finger_number, &data_position);
				diff = current_angle - desired_angle;
			}

			for(j = 0; j < num_triggers; j++){
				if(triggers[j].triggered || triggers[j].actuator_number != actuator_number){
					continue;
				}
				
				if(fabs(triggers[j].actuator_position - current_angle) < epsilon){
					cout << "Found candidate actuator " << actuator_number << endl;
					triggers[j].triggered = true;
					if(triggers[j].move_actuator){
						offset = triggers[j].new_actuator_number;
						angles[offset] = triggers[j].new_actuator_position;
					}else if(triggers[j].move_finger){
						offset = triggers[j].new_finger_number + NUM_ACTUATORS;
						angles[offset] = triggers[j].new_finger_position;
					}
					finished[offset] = false;
				}
			}

			to_move = 0;
			//cout << "joint: " << i << " " << current_angle << " - " << desired_angle << " = " << diff << endl;
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
				//cout << (is_actuator ? "\tFinished actuator " : "\tFinished finger ") << i << endl;
				finished[i] = true;
			}else if(is_actuator){
				set_actuator_movement(actuator_number, &point_to_send, to_move);
			}else if(is_finger){
				set_finger_movement(finger_number, &point_to_send, to_move);
				//cout << "setting finger movement " << to_move << endl;
			}

			if(!finished[i] && current_angle == current_angles[i]){
				tries_without_change++;
			}else if(!finished[i] && current_angle != current_angles[i]){
				tries_without_change = 0;
				current_angles[i] = current_angle;
			}
			
		}
		//cout << "sending point " << num_points_sent << ", ineffectual tries: " << tries_without_change << endl;
		num_points_sent++;
		MySendBasicTrajectory(point_to_send);
		usleep(10000);
	}
}


void print_state(struct viz_thread_args *viz_args){
	int angles[NUM_COMPONENTS];
	load_current_angles(angles);
	cout << "(JACO) Actuators (degrees):" << endl;
	for(int i = 0; i < NUM_ACTUATORS; i++){
		cout << "\t" << i << " " << angles[i] << endl;
	}
	
	struct cartesian_xyz xyz_thetas;
	load_current_cartesian_position(&xyz_thetas);
	cout << "(JACO) Cartesian Positions (meters, degrees):" << endl;
	cout << "\tX:      " << xyz_thetas.x << endl;
	cout << "\tY:      " << xyz_thetas.y << endl;
	cout << "\tZ:      " << xyz_thetas.z << endl;
	cout << "\tThetaX: " << xyz_thetas.theta_x << endl;
	cout << "\tThetaY: " << xyz_thetas.theta_y << endl;
	cout << "\tThetaZ: " << xyz_thetas.theta_z << endl;

	cout << "(JACO) Fingers:" << endl;
	for(int i = NUM_ACTUATORS; i < NUM_ACTUATORS + NUM_FINGERS; i++){
		cout << "\t" << i << " " << angles[i] << endl;
	}

	cout << "(KINECT) JACO Tags:" << endl;
	for(int i = 0; i < viz_args->num_jaco_tags; i++){
		cout << "\t" << i << " " << viz_args->jaco_tag_xyz[i].x << "\t" << viz_args->jaco_tag_xyz[i].y << "\t" << viz_args->jaco_tag_xyz[i].z << "\tDistance: " << viz_args->jaco_distances[i] << endl;
	}
	
	cout << "Objects of interest:" << endl;
	for(int i = 0; i < viz_args->num_objects; i++){
		cout << "(KINECT)\t" << i << " " << viz_args->object_xyz[i].x << "\t" << viz_args->object_xyz[i].y << "\t" << viz_args->object_xyz[i].z << "\tDistance: " << viz_args->object_distances[i] << endl;
		cout << "rostopic pub /iral/jaco/input --once std_msgs/String \"" << HOVER_AT_THEN_RETURN << "," << viz_args->object_xyz[i].x << "," << viz_args->object_xyz[i].y << "," << viz_args->object_xyz[i].z << "\"" << endl;
		
		if(viz_args->num_jaco_tags > 0){
			translate_kinect_to_jaco(&xyz_thetas, &(viz_args->object_xyz[i]), &(viz_args->jaco_tag_xyz[0]));
			cout << "(JACO)\tcart goto " << xyz_thetas.x << " " << xyz_thetas.y << " " << xyz_thetas.z << " " << M_PI << " 0 0" << endl;
		}
	}
}














#endif


 

