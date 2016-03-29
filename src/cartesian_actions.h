
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

void cartesian_straighten(struct thread_args *args);



void do_cartesian_action(struct thread_args *args, bool blocking){
	if(!args->arm_has_moved){
		cout << "doing action" << endl;
		args->arm_has_moved = true;
		cartesian_straighten(args);
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


void reset_cartesian_point_to_send(TrajectoryPoint *point_to_send){
	point_to_send->InitStruct();
	point_to_send->Position.Type = CARTESIAN_VELOCITY;
	
	point_to_send->Position.CartesianPosition.X = 0;
	point_to_send->Position.CartesianPosition.Y = 0;
	point_to_send->Position.CartesianPosition.Z = 0;
	point_to_send->Position.CartesianPosition.ThetaX = 0;
	point_to_send->Position.CartesianPosition.ThetaY = 0;
	point_to_send->Position.CartesianPosition.ThetaZ = 0;

	point_to_send->Position.Fingers.Finger1 = 0;
	point_to_send->Position.Fingers.Finger2 = 0;
	point_to_send->Position.Fingers.Finger3 = 0;
}

void cartesian_straighten(struct thread_args *args){
	cout << "Running cartesian straighten" << endl;
	args->xyz_thetas.x = 0.068451;
	args->xyz_thetas.y = -0.0126348;
	args->xyz_thetas.z = 1.18026;
	args->xyz_thetas.theta_x = 0.0296927;
	args->xyz_thetas.theta_y = 0.0103306;
	args->xyz_thetas.theta_z = -3.09015;
	
	do_cartesian_action(args, true);
}

double update_cartesian_movement(double current_position, double goal_coord, bool *all_done){
	return 0.1f;
	double diff = current_position - goal_coord;
	double to_move = 0.0;
	double xyz_speed = 0.15;
	double angle_epsilon = 0.01;
	double max_speed;
	if(diff > angle_epsilon){
		cout << "diff: " << diff << endl;
		max_speed = fabs(diff) < xyz_speed ? diff : xyz_speed;
		to_move = ((diff > 0 ? -1 : 1) * max_speed);
		cout << "diff: " << diff << ", to move: " << to_move << endl;
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
	while(!arrived && keepRunning){
		arrived = true;
		reset_cartesian_point_to_send(&point_to_send);
		(*MyGetCartesianCommand)(cartesian_position);
		
		//point_to_send.Position.CartesianPosition.X += update_cartesian_movement(cartesian_position.Coordinates.X, goal_xyz_thetas->x, &arrived);
		//point_to_send.Position.CartesianPosition.Y += update_cartesian_movement(cartesian_position.Coordinates.Y, goal_xyz_thetas->y, &arrived);
		//point_to_send.Position.CartesianPosition.Z += update_cartesian_movement(cartesian_position.Coordinates.Z, goal_xyz_thetas->z, &arrived);

                point_to_send.Position.CartesianPosition.Y = -0.15; //Move along Y axis at 20 cm per second


                for(int i = 0; i < 200; i++){
                        //We send the velocity vector every 5 ms as long as we want the robot to move along that vector.
                        MySendBasicTrajectory(point_to_send);
                        usleep(5000);
                }



		cout << "movement: (" << point_to_send.Position.CartesianPosition.X << "," << point_to_send.Position.CartesianPosition.Y << "," << point_to_send.Position.CartesianPosition.Z << ")" << endl;

		cout << "position: (" << cartesian_position.Coordinates.X << "," << cartesian_position.Coordinates.Y << "," << cartesian_position.Coordinates.Z << ")" << endl;
		
		//MySendBasicTrajectory(point_to_send);
                //        usleep(5000);
		break;
	}
}

void go_to_cartesian_position(struct xyz *xyz){
	
	
}








#endif




