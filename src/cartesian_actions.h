
#ifndef _CARTESIANACTIONSH_
#define _CARTESIANACTIONSH_


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

void cartesian_home(struct thread_args *args);



void do_cartesian_action(struct thread_args *args, bool blocking){
	if(!args->arm_has_moved){
		cout << "initializing" << endl;
		args->arm_has_moved = true;
		MyInitFingers();
		MyMoveHome();
		cout << "done, continuing" << endl;
	}
	
	args->completed_move = false;
	args->wake_up_cartesian = true;
	while(blocking && !args->completed_move){
		usleep(100);
	}
}

void load_current_cartesian_position(struct cartesian_xyz *xyz_thetas){
	int i;
		
	memset(xyz_thetas, 0, sizeof(struct cartesian_xyz));
	
	(*MyGetCartesianCommand)(cartesian_position);
	xyz_thetas->x = cartesian_position.Coordinates.X;
	xyz_thetas->y = cartesian_position.Coordinates.Y;
	xyz_thetas->z = cartesian_position.Coordinates.Z;
	xyz_thetas->theta_x = cartesian_position.Coordinates.ThetaX;
	xyz_thetas->theta_y = cartesian_position.Coordinates.ThetaY;
	xyz_thetas->theta_z = cartesian_position.Coordinates.ThetaZ;
}


void load_current_cartesian_coords(TrajectoryPoint *point_to_send){
	(*MyGetCartesianCommand)(cartesian_position);
	point_to_send->Position.CartesianPosition.X = cartesian_position.Coordinates.X;
	point_to_send->Position.CartesianPosition.Y = cartesian_position.Coordinates.Y;
	point_to_send->Position.CartesianPosition.Z = cartesian_position.Coordinates.Z;
	point_to_send->Position.CartesianPosition.ThetaX = cartesian_position.Coordinates.ThetaX;
	point_to_send->Position.CartesianPosition.ThetaY = cartesian_position.Coordinates.ThetaY;
	point_to_send->Position.CartesianPosition.ThetaZ = cartesian_position.Coordinates.ThetaZ;

	(*MyGetCartesianCommand)(cartesian_position);
	point_to_send->Position.Fingers.Finger1 = cartesian_position.Fingers.Finger1;
	point_to_send->Position.Fingers.Finger2 = cartesian_position.Fingers.Finger2;
	point_to_send->Position.Fingers.Finger3 = cartesian_position.Fingers.Finger3;
}

void reset_cartesian_point_to_send(TrajectoryPoint *point_to_send){
	point_to_send->InitStruct();
	point_to_send->Position.Type = CARTESIAN_POSITION;
	

	load_current_cartesian_coords(point_to_send);
}

void cartesian_home(struct thread_args *args){
	MyMoveHome();
	
	cout << "Running cartesian go home" << endl;
	args->xyz_thetas.x = 0.223358;
	args->xyz_thetas.y = -0.203183;
	args->xyz_thetas.z = 0.505893;
	args->xyz_thetas.theta_x = 1.78079;
	args->xyz_thetas.theta_y = 1.22449;
	args->xyz_thetas.theta_z = -0.229752;
	
	do_cartesian_action(args, true);
}

double update_cartesian_movement(double current_position, double goal_coord, bool *all_done){
	double diff = current_position - goal_coord;
	double to_move = 0.0;
	double xyz_speed = 0.15;
	double location_epsilon = 0.05;
	double max_speed;
	//cout << "diff: " << diff << endl;
	if(fabs(diff) > location_epsilon){
		max_speed = fabs(diff) < xyz_speed ? diff : xyz_speed;
		to_move = ((diff > 0 ? -1 : 1) * max_speed);
		//cout << "diff: " << diff << ", to move: " << to_move << endl;
		(*all_done) = false;
	}
	return to_move;	
}

void layered_cartesian_move(struct cartesian_xyz *goal_xyz_thetas){

	TrajectoryPoint point_to_send;
	
	int desired_position = 0;
	double diff;
	bool arrived = false;
	
	// higher allowance for fingers
	int i, j, position_number, finger_number;
	CartesianPosition currentCommand;
	
	reset_cartesian_point_to_send(&point_to_send);
	
	struct cartesian_xyz last_xyz_thetas;
	
	bool point_sent = false;
	int max_tries = 5;
	int tries = 0;
	while(!arrived && keepRunning){
		arrived = true;
		(*MyGetCartesianCommand)(cartesian_position);
		
		update_cartesian_movement(cartesian_position.Coordinates.X, goal_xyz_thetas->x, &arrived);
		update_cartesian_movement(cartesian_position.Coordinates.Y, goal_xyz_thetas->y, &arrived);
		update_cartesian_movement(cartesian_position.Coordinates.Z, goal_xyz_thetas->z, &arrived);

		update_cartesian_movement(cartesian_position.Coordinates.ThetaX, goal_xyz_thetas->theta_x, &arrived);
		update_cartesian_movement(cartesian_position.Coordinates.ThetaY, goal_xyz_thetas->theta_y, &arrived);
		update_cartesian_movement(cartesian_position.Coordinates.ThetaZ, goal_xyz_thetas->theta_z, &arrived);
		
		
		//cout << "desired: (" << goal_xyz_thetas->x << "," << goal_xyz_thetas->y << "," << goal_xyz_thetas->z << ") theta: (" << goal_xyz_thetas->theta_x << "," << goal_xyz_thetas->theta_y << "," << goal_xyz_thetas->theta_z << ")" << endl;
		
		//cout << "position: (" << cartesian_position.Coordinates.X << "," << cartesian_position.Coordinates.Y << "," << cartesian_position.Coordinates.Z << ") theta: (" << cartesian_position.Coordinates.ThetaX << "," << cartesian_position.Coordinates.ThetaY << "," << cartesian_position.Coordinates.ThetaZ << ")" << endl;

		if(!point_sent){
			point_to_send.Position.CartesianPosition.X = goal_xyz_thetas->x;
			point_to_send.Position.CartesianPosition.Y = goal_xyz_thetas->y;
			point_to_send.Position.CartesianPosition.Z = goal_xyz_thetas->z;
			point_to_send.Position.CartesianPosition.ThetaX = goal_xyz_thetas->theta_x;
			point_to_send.Position.CartesianPosition.ThetaY = goal_xyz_thetas->theta_y;
			point_to_send.Position.CartesianPosition.ThetaZ = goal_xyz_thetas->theta_z;

		
		        MySendBasicTrajectory(point_to_send);
			point_sent = true;
		}else if(last_xyz_thetas.x == cartesian_position.Coordinates.X &&
				last_xyz_thetas.y == cartesian_position.Coordinates.Y &&
				last_xyz_thetas.z == cartesian_position.Coordinates.Z &&
				last_xyz_thetas.theta_x == cartesian_position.Coordinates.ThetaX &&
				last_xyz_thetas.theta_y == cartesian_position.Coordinates.ThetaY &&
				last_xyz_thetas.theta_z == cartesian_position.Coordinates.ThetaZ){
			cout << "Not moving, possibly stuck" << endl;
			
			tries++;
		}else{
			tries = 0;
		}

		if(tries >= max_tries){
			cout << "Finished moving, stopping" << endl;
			break;
		}
		last_xyz_thetas.x = cartesian_position.Coordinates.X;
		last_xyz_thetas.y = cartesian_position.Coordinates.Y;
		last_xyz_thetas.z = cartesian_position.Coordinates.Z;
		last_xyz_thetas.theta_x = cartesian_position.Coordinates.ThetaX;
		last_xyz_thetas.theta_y = cartesian_position.Coordinates.ThetaY;
		last_xyz_thetas.theta_z = cartesian_position.Coordinates.ThetaZ;
		
		// half-second sleep
		usleep(500000);
	}
}





#endif




