#ifndef _UTILH_
#define _UTILH_

#include <string.h>
#include <iostream>
#include <stdexcept>

#include "Kinova.API.CommLayerUbuntu.h"
#include "KinovaTypes.h"

#include <mlpack/methods/kmeans/kmeans.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>

#define KINECT_HD_TOPIC "/kinect2/hd/points"
#define KINECT_QHD_TOPIC "/kinect2/qhd/points"
#define KINECT_SD_TOPIC "/kinect2/sd/points"

#define DEFAULT_KINECT_TOPIC KINECT_SD_TOPIC

#define INPUT_TOPIC "/iral/jaco/input"

#define COMMAND_DELAY 3

#define NUM_ACTUATORS 6
#define NUM_FINGERS 3
#define NUM_COMPONENTS NUM_ACTUATORS+NUM_FINGERS

// LEAVE THIS VALUE AT 0, TOO EXPENSIVE DURING THE DBSCAN, MEM ALLOC ERROR
#define DEFAULT_ADDITIONAL_COLOR_MATCH_FRAMES_TO_COMBINE 0


#define DEFAULT_NUM_JACO_ARMS_IN_SCENE 1
#define DEFAULT_NUM_OBJECTS_IN_SCENE 1

#define DEFAULT_SKIP_FRAMES 0

#define DEFAULT_VISIBLE_ANGLE 30

#define DEFAULT_HIGHLIGHT_VISIBLE_AREA false

#define DEFAULT_MAX_INTERESTED_DISTANCE 2.0
#define DEFAULT_MIN_INTERESTED_DISTANCE 0.1

#define MAX_INTERESTED_DISTANCE_INTERVAL 0.25

#define HEIGHT_OFFSET_ADJUSTMENT 0.2

#define DEFAULT_FIND_ARM false

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


typedef enum {
	KEEP_ALIVE = 0,

	// receives just X,Y,Z in Kinect-point-cloud-space
	MOVE_TO_XYZ,
	HOVER_OVER_XYZ,

	GO_HOME,
	// receives just X,Y from kinect's 2d RGB image
	HOVER_OVER_XY,

	NUMBER_OF_MESSAGE_TYPES
} ros_input_message_type;


struct rgb{
	short r;
	short g;
	short b;
};


#define MAX_COLORS_PER_ITEM 3
struct rgb_set{
	int num_colors;
	struct rgb colors[MAX_COLORS_PER_ITEM];
};

typedef enum{
	STARTING = 0,
	NORMAL,

	NUMBER_OF_POINT_TYPES
} point_type;
struct point_pairs{
	vector<double> parent;
	vector<double> candidate;
	point_type type;
	int up_iterations;
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
	bool find_arm;

	bool highlight_table;

	int skip_frames;

	char *kinect_topic;
	
	int num_objects_in_scene;
	int num_jaco_arms_in_scene;

	double max_interested_distance;
	object_detection_algorithm detection_algorithm;

	struct rgb_set orange_bottle_colors;

	pcl::PointCloud<pcl::PointXYZRGB> *cloud;
};

struct ros_input{
	int *argc;
	char ***argv;
	bool terminate;

	struct viz_thread_args *viz_args;
	
	ros_input_message_type msg;
	
	struct xyz move_to;
	bool ready;
	bool completed;
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

	// get input from ros action subscriber
	struct ros_input *ros_input;
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
	xyz_thetas->y = obj_xyz.at(1) + 0.04; // adjustment to center opposing fingers over bottle.
	xyz_thetas->z = obj_xyz.at(2);

	xyz_thetas->theta_x = M_PI;
	xyz_thetas->theta_y = 0;
	xyz_thetas->theta_z = 0;

}





std::string exec(const char* cmd) {
	char buffer[128];
	std::string result = "";
	FILE* pipe = popen(cmd, "r");
	if (!pipe) throw std::runtime_error("popen() failed!");
	try {
		while (!feof(pipe)) {
			if (fgets(buffer, 128, pipe) != NULL)
				result += buffer;
		}
	} catch (...) {
		pclose(pipe);
		throw;
	}
	pclose(pipe);
	return result;
}

void get_onscreen_color(struct rgb *color){
	std::string result = exec("grabc 2>&1 | grep -v \"#\"");
	// from http://www.cplusplus.com/reference/string/string/c_str/
	char * cstr = new char [result.length()+1];
	std::strcpy (cstr, result.c_str());
	char * p = std::strtok (cstr,",");
	color->r = atoi(p);
	p = std::strtok(NULL,",");
	color->g = atoi(p);	
	p = std::strtok(NULL,",");
	color->b = atoi(p);
	
	cout << "color: " << color->r << "-" << color->g << "-" << color->b << endl;
}

#endif

