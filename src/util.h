#ifndef _UTILH_
#define _UTILH_

#include <string.h>

#include "Kinova.API.CommLayerUbuntu.h"
#include "KinovaTypes.h"

#include <mlpack/methods/kmeans/kmeans.hpp>

#define COMMAND_DELAY 3

#define NUM_ACTUATORS 6
#define NUM_FINGERS 3
#define NUM_COMPONENTS NUM_ACTUATORS+NUM_FINGERS

#define DEFAULT_ADDITIONAL_COLOR_MATCH_FRAMES_TO_COMBINE 4
#define DEFAULT_NUM_JACO_ARMS_IN_SCENE 1
#define DEFAULT_NUM_OBJECTS_IN_SCENE 1

#define DEFAULT_VISIBLE_ANGLE 20

#define DEFAULT_HIGHLIGHT_VISIBLE_AREA true

#define DEFAULT_MAX_INTERESTED_DISTANCE 2.0
#define DEFAULT_MIN_INTERESTED_DISTANCE 0.6

#define MAX_INTERESTED_DISTANCE_INTERVAL 0.25

#define HEIGHT_OFFSET_ADJUSTMENT 0.2

using namespace std;

typedef enum {
	KMEANS = 0,
	HISTOGRAM,

	NUMBER_OF_DETECTION_ALGORITHMS
} object_detection_algorithm;

const char* const detection_algorithm_names[] = {"kmeans", "histogram", 0};
#define DEFAULT_OBJECT_DETECTION_ALG HISTOGRAM

AngularPosition current_command;
AngularPosition data_command;
AngularPosition data_position;

CartesianPosition cartesian_position;

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

struct cartesian_xyz{
	double x;
	double y;
	double z;
	
	double theta_x;
	double theta_y;
	double theta_z;
	
	int finger_1;
	int finger_2;
	int finger_3;
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
	int additional_color_match_frames_to_combine;

	bool highlight_visible_area;
	int visible_angle;
	
	int num_objects_in_scene;
	int num_jaco_arms_in_scene;

	double max_interested_distance;
	object_detection_algorithm detection_algorithm;
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
	
	// Track what the goal cartesian angles are for the arm/fingers
	struct cartesian_xyz xyz_thetas;
	
	// Track what the cartesian angles for the arm/fingers were when object was grasped
	bool object_grasped;
	struct cartesian_xyz original_object_jaco_space_xyz_thetas;
	
	// Triggers that can be matched during move execution
	int num_triggers;
	struct actuator_trigger *triggers;

	// track if arm state is dirty, avoid straightening/shutdown if never moved.	
	bool arm_has_moved;
	
	// flag to wake the thread back up
	bool wake_up;
	bool wake_up_cartesian;
	
	// thread will set this to true when current instructions are met
	bool completed_move;
	
	// shutdown flag, terminate pthread
	bool shutdown;

	// pass info to the visualizer
	struct viz_thread_args *viz_args;
};


void print_matrix(arma::mat *mat){
	int i,j;
	for(i = 0; i < mat->n_rows; i++){
		for(j = 0; j < mat->n_cols; j++){
			cout << mat->at(i,j) << ", ";
		}
		cout << endl;
	}

}

void translate_kinect_to_jaco(struct cartesian_xyz *xyz_thetas, struct xyz *object_xyz, struct xyz *jaco_xyz){
	//cout << "identity" << endl;
	arma::mat translation;
	translation 	<< 1 << 0 << 0 << 0 << arma::endr
			<< 0 << 1 << 0 << 0 << arma::endr
			<< 0 << 0 << 1 << 0 << arma::endr
			<< 0 << 0 << 0 << 1 << arma::endr;
	//print_matrix(&translation);
	
	
	//cout << "translate" << endl;
	translation.at(0,3) = -1 * jaco_xyz->x;
	translation.at(1,3) = -1 * jaco_xyz->y;
	translation.at(2,3) = -1 * jaco_xyz->z;
	//print_matrix(&translation);
	
	//cout << "scale" << endl;
	arma::mat scale(4, 4, arma::fill::eye);
	//scale.at(2,2) = -1; // y -1 because y is coming towards the kinect
	//print_matrix(&scale);


	//cout << "rotation around x" << endl;
	double radians = M_PI_2;
	arma::mat rotation;
	rotation 	<< 1 << 0 		<< 0 			<< 0 << arma::endr
			<< 0 << cos(radians)	<< -1*sin(radians)	<< 0 << arma::endr
			<< 0 << sin(radians) 	<< cos(radians)		<< 0 << arma::endr
			<< 0 << 0 		<< 0 			<< 1 << arma::endr;
	
	//print_matrix(&rotation);
	

	arma::mat transformation(4, 4, arma::fill::eye);
	transformation = transformation * rotation * translation * scale;
	//cout << "transform" << endl;
	//print_matrix(&transformation);
	

	//cout << "Vector" << endl;
	arma::vec obj_xyz = arma::ones<arma::vec>(4);
	obj_xyz.at(0) = object_xyz->x;
	obj_xyz.at(1) = object_xyz->y;
	obj_xyz.at(2) = object_xyz->z;
	//print_matrix(&obj_xyz);

	obj_xyz = transformation * obj_xyz;
	
	//cout << "transformed" << endl;
	//print_matrix(&obj_xyz);

	//cout << "adjusted jaco Z for tag height offset" << endl;
	obj_xyz.at(2) += HEIGHT_OFFSET_ADJUSTMENT; //2 * 0.0906932; // measured by moving hand to tag level
	//print_matrix(&obj_xyz);
	
	xyz_thetas->x = obj_xyz.at(0);
	xyz_thetas->y = obj_xyz.at(1);
	xyz_thetas->z = obj_xyz.at(2);

	xyz_thetas->theta_x = M_PI;
	xyz_thetas->theta_y = 0;
	xyz_thetas->theta_z = 0;

}

#endif

