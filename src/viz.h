#ifndef _VIZH_
#define _VIZH_

#include <mlpack/methods/kmeans/kmeans.hpp>

#include <vector>
#include <pthread.h>
#include <cmath>
#include <cfloat>

#include "opencv2/core/core.hpp"
#include "opencv2/flann/miniflann.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/photo/photo.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/ml/ml.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"

#include "opencv2/core/core_c.h"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/imgproc/imgproc_c.h"

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace mlpack::kmeans;

struct rgb{
	short r;
	short g;
	short b;
};


// orange color: #ef5e25
struct rgb orange = {239, 94, 37};

// blue tag color: #2499bc
struct rgb blue_tag = {36, 153, 188};

// bottle color: #45c5d2
struct rgb bottle = {69, 197, 210};

// green cylinder color: #52c77f
struct rgb green_cylinder = {82, 199, 127};

// pixel shading color for matches
struct rgb match_color = {0xff, 0xd7, 0x00};

// pixel shading color for jaco tag, red
struct rgb jaco_match_color = {0xff, 0, 0};


bool colors_are_similar(struct rgb *x, struct rgb *y){
	double distance = sqrt( (x->r - y->r)*(x->r - y->r) + (x->g - y->g)*(x->g - y->g) + (x->b - y->b)*(x->b - y->b) );
	// epsilon subject to change here, was trial and error early on.
	return distance < 50;
}


static const std::string OPENCV_WINDOW = "Image window";
#define BATCH_SIZE 1000;


double euclid_distance_3d(vector<double> a, vector<double> b){
	// compute integer distance between two points in 3d
	return (double) sqrtf((a.at(0) - b.at(0)) * (a.at(0) - b.at(0)) + (a.at(1) - b.at(1)) * (a.at(1) - b.at(1)) + (a.at(2) - b.at(2)) * (a.at(2) - b.at(2)) );
}

double euclid_distance_3d_not_vec_double(double *a, double *b){
	// compute integer distance between two points in 3d as a[] and b[]
	return (double) sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2]) );
}

int euclid_distance_3d_not_vec(double *a, double *b){
	// compute integer distance between two points in 3d as a[] and b[]
	return (int) sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2]) );
}

double vector_length_3d(double *xyz){
	return (double) sqrtf(xyz[0]*xyz[0] + xyz[1]*xyz[1] + xyz[2]*xyz[2]);
}

double vector_length_3d_struct(struct xyz *xyz){
	return (double) sqrtf(xyz->x*xyz->x + xyz->y*xyz->y + xyz->z*xyz->z);
}

int color_normalize(double dist, double max, int span){
	return (int)((dist / max) * span);
}

void get_xyz_from_xyzrgb(int x, int y, pcl::PointCloud<pcl::PointXYZRGB> *cloud, double *xyz){
	pcl::PointXYZRGB *point = &(cloud->at(x, y));
	xyz[0] = point->x;
	xyz[1] = point->y;
	xyz[2] = point->z;
}

void apply_distance_filter(cv::Mat *im_matrix, int x, int y, pcl::PointCloud<pcl::PointXYZRGB> *cloud){
	double dist;
	double xyz[3];
	get_xyz_from_xyzrgb(x, y, cloud, xyz);
	
	dist = vector_length_3d(xyz);
	short color = color_normalize(dist, 3.0, 255);
	color = MIN(255, color);
	color = MAX(0, color);
	im_matrix->at<cv::Vec3b>(y,x)[0] = color;
	im_matrix->at<cv::Vec3b>(y,x)[1] = color;
	im_matrix->at<cv::Vec3b>(y,x)[2] = color;
}

bool is_valid_xyz(double *xyz){
	return !std::isnan(xyz[0]) && !std::isnan(xyz[1]) && !std::isnan(xyz[2]);
}

bool do_pixel_test(int x, int y, cv::Mat *image, struct rgb *desired, vector< vector<double> > *matches_2d, vector< vector<double> > *matches_3d, pcl::PointCloud<pcl::PointXYZRGB> *cloud){
	struct rgb test;
	test.r = (*image).at<cv::Vec3b>(y,x)[2];
	test.g = (*image).at<cv::Vec3b>(y,x)[1];
	test.b = (*image).at<cv::Vec3b>(y,x)[0];
	
	int matches_size = matches_2d->size();
	bool match = false;
	if(colors_are_similar(desired, &test)){
		double xyz[3];
		get_xyz_from_xyzrgb(x, y, cloud, xyz);
		
		if(!is_valid_xyz(xyz)){
			return false;
		}
		
		matches_3d->push_back( vector<double>(3) );
		matches_3d->at(matches_size).at(0) = xyz[0];
		matches_3d->at(matches_size).at(1) = xyz[1];
		matches_3d->at(matches_size).at(2) = xyz[2];
		
		matches_2d->push_back( vector<double>(2) );
		matches_2d->at(matches_size).at(0) = (int) x;
		matches_2d->at(matches_size).at(1) = (int) y;
		
		//cout << "3d xyz: " << xyz[0] << "," << xyz[1] << "," << xyz[2] << endl;
		//cout << "3d match: " << matches_3d->at(matches_size).at(0) << "," << matches_3d->at(matches_size).at(1) << "," << matches_3d->at(matches_size).at(2) << endl;
		//cout << "2d match: " << matches_2d->at(matches_size).at(0) << "," << matches_2d->at(matches_size).at(1) << endl;
		match = true;
	}
	return match;
}

// Function sorts vectors with (centroid id, number of members) pairs
bool sort_centroid_members(vector< double > a, vector< double > b) { return a[1] > b[1]; }

class ImageConverter{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	ros::Subscriber pcl_sub_;
	int frames;
	struct viz_thread_args *args;

	// store the centroids in both xyz and 2d image
	vector< vector<double> > object_centroids_2d;
	vector< vector<double> > object_centroids_3d;

	vector< vector< vector<double> > > object_matched_points_2d_previous_rounds;
	vector< vector< vector<double> > > object_matched_points_3d_previous_rounds;

	vector< vector<double> > jaco_tag_centroids_2d;
	vector< vector<double> > jaco_tag_centroids_3d;

	vector< vector< vector<double> > > jaco_tag_matched_points_2d_previous_rounds;
	vector< vector< vector<double> > > jaco_tag_matched_points_3d_previous_rounds;
	
	pthread_mutex_t centroid_mutex;

	public:
	ImageConverter() : 
	it_(nh_){
		frames = 0;
		// Create a ROS subscriber for the input point cloud, contains XYZ, RGB
		pcl_sub_ = nh_.subscribe ("/kinect2/qhd/points", 1, &ImageConverter::cloudCb, this);
	
		object_centroids_2d.clear();
		object_centroids_3d.clear();
	

		object_matched_points_2d_previous_rounds.clear();
		object_matched_points_3d_previous_rounds.clear();

		jaco_tag_matched_points_2d_previous_rounds.clear();
		jaco_tag_matched_points_3d_previous_rounds.clear();

		jaco_tag_centroids_2d.clear();
		jaco_tag_centroids_3d.clear();
		
		cv::namedWindow(OPENCV_WINDOW);
	}

	~ImageConverter(){
		cv::destroyWindow(OPENCV_WINDOW);
	}

	void set_args(struct viz_thread_args *viz_args){
		args = viz_args;
	}

	bool find_match_by_color(cv::Mat *im_matrix, pcl::PointCloud<pcl::PointXYZRGB> *cloud, int x, int y, vector< vector<double> > *matched_points_2d, vector< vector<double> > *matched_points_3d, struct rgb *object, bool verbose){
		pcl::PointXYZRGB *point;
		
		// Cloud is (columns, rows)
		point = &cloud->at(x, y);

		Eigen::Vector3i rgb = point->getRGBVector3i();

		// im_matrix is (rows, columns)
		im_matrix->at<cv::Vec3b>(y,x)[0] = rgb[2];
		im_matrix->at<cv::Vec3b>(y,x)[1] = rgb[1];
		im_matrix->at<cv::Vec3b>(y,x)[2] = rgb[0];
		
		bool match = do_pixel_test(x, y, im_matrix, object, matched_points_2d, matched_points_3d, cloud);
		
		if(args->draw_depth_filter){
			apply_distance_filter(im_matrix, x, y, cloud);
		}
		return match;
	}

	double compute_centroid_error(vector<double> *centroid, int centroid_number, arma::Row<size_t> *assignments, vector< vector<double> > *samples, int *num_assignments){
		// find the samples in the cluster with the given centroid and compute mean distance to the centroid.
		double distance_sum = 0;
		(*num_assignments) = 0;
		for(int i = 0; i < assignments->size(); i++){
			if(assignments->at(i) != centroid_number){
				continue;
			}
			(*num_assignments)++;
			distance_sum += euclid_distance_3d(samples->at(i), *centroid);
		}
		if(distance_sum > 0){
			distance_sum = distance_sum / *num_assignments;
		}
		
		return distance_sum;
		
	}

	void build_centroid(vector< vector<double> > *centroids, arma::mat *k_centroids, int num_elements){
		int num_centroids = centroids->size();
		centroids->push_back(vector<double>(num_elements));
		for(int element_offset = 0; element_offset < num_elements; element_offset++){
			centroids->at(num_centroids).at(element_offset) = (*k_centroids)[num_centroids * num_elements + element_offset];
		}
	}

	bool too_many_clusters(double new_val, double old_val){
		// TODO: figure out a good way to return true on elbow function
		// https://en.wikipedia.org/wiki/Determining_the_number_of_clusters_in_a_data_set
		
		// naive way: return true if 2/3 of old_val is less than new_val, i.e. not a big enough drop in error.
		//cout << "\ttesting new " << new_val << " vs old " << old_val << endl;
		return (old_val * 0.66667) < new_val;
	}

	void kmeans_cluster_and_centroid(vector< vector<double> > *samples, vector< vector<double> > *centroids, int max_centroids_to_try, int absolute_max_centroids, bool verbose){
		// Largely duped and modified from http://www.mlpack.org/docs/mlpack-2.0.1/doxygen.php?doc=kmtutorial.html#kmeans_kmtut
		// http://arma.sourceforge.net/docs.html
		centroids->clear();
		if(samples->size() == 0){
			return;
		}
		// The dataset we are clustering.
		arma::mat data;
		data.zeros(samples->at(0).size(), samples->size()); // Column major, number of elements in a match (2d = 2, 3d = 3, etc) rows for xyz, n columns
		for(int i = 0; i < samples->size(); i++){
			for(int j = 0; j < samples->at(i).size(); j++){
				data.at(j, i) = samples->at(i).at(j);
			}
		}
		
		/*
		data << 0 << 0 << 0 << 0 << 5 << 5 << 5 << 5 << arma::endr
		     << 0 << 0 << 1 << 1 << 0 << 0 << 1 << 1 << arma::endr
		     << 0 << 1 << 0 << 1 << 0 << 1 << 0 << 1 << arma::endr;
		// results in (2.5,0.5,0.5) for a single centroid.
		
		*/
		//cout << "Data rows: " << data.n_rows << ", cols: " << data.n_cols << " from " << matches->size() << " points" << endl;
		
		
		
		// The number of clusters we are getting.
		
		// The assignments will be stored in this vector.
		arma::Row<size_t> assignments;
		// The centroids will be stored in this matrix.
		arma::mat k_centroids;
		// Initialize with the default arguments.
		KMeans<> k;
		
		double centroid_error;
		int num_assignments;

		int ideal_centroid_count = -1;
		double error_sum_this_round, error_sum_last_round;
		
		for(int num_centroids = 1; num_centroids < max_centroids_to_try; num_centroids++){
			centroids->clear();
			if(samples->size() < num_centroids){
				continue;
			}
			
			k.Cluster(data, num_centroids, assignments, k_centroids);
			
			error_sum_this_round = 0;
			
			for(int this_centroid = 0; this_centroid < num_centroids; this_centroid++){
				build_centroid(centroids, &k_centroids, data.n_rows);

				centroid_error = compute_centroid_error(&(centroids->at(this_centroid)), this_centroid, &assignments, samples, &num_assignments);

				error_sum_this_round += centroid_error;

				if(verbose){
					cout << "(" << num_centroids << ") centroid " << this_centroid << ": ";
					for(int element_offset = 0; element_offset < data.n_rows; element_offset++){
						cout << centroids->at(this_centroid).at(element_offset) << " ";
					}
					cout << " has error: " << centroid_error << " and contains " << num_assignments << " samples" << " samples (running error: " << error_sum_this_round << endl;
				}
				

			}
			// Have to try at least 1 centroid before we could break for elbow, since not enough comparisons
			if(num_centroids > 1 && too_many_clusters(error_sum_this_round, error_sum_last_round)){
				ideal_centroid_count = num_centroids;
				//cout << "FOUND TOO MANY" << endl;
				break;
			}
			error_sum_last_round = error_sum_this_round;
		}
		if(ideal_centroid_count == -1){
			ideal_centroid_count = max_centroids_to_try;
		}
		
		vector< vector< double > > centroid_sizes;
		int num_so_far = 0;
		
		if(samples->size() >= ideal_centroid_count){
			k.Cluster(data, ideal_centroid_count, assignments, k_centroids);
			error_sum_this_round = 0;
			centroids->clear();
			int min_points_per_cluster = 100; // completely arbitrary value here, testing...
			int num_centroids;
			for(int this_centroid = 0; this_centroid < ideal_centroid_count; this_centroid++){
				build_centroid(centroids, &k_centroids, data.n_rows);
				num_centroids = centroids->size();
				centroid_error = compute_centroid_error(&(centroids->at(num_centroids - 1)), this_centroid, &assignments, samples, &num_assignments);
				if(num_assignments < min_points_per_cluster){
					centroids->erase(centroids->begin() + num_centroids - 1);
					continue;
				}
				error_sum_this_round += centroid_error;
				
				centroid_sizes.push_back( vector< double >(2) );
				centroid_sizes.at(num_so_far).at(0) = num_so_far;
				centroid_sizes.at(num_so_far).at(1) = num_assignments;
				num_so_far++;
				
				if(verbose){
					cout << "\tcentroid " << num_centroids-1 << ": ";
					for(int element_offset = 0; element_offset < data.n_rows; element_offset++){
						cout << centroids->at(num_centroids - 1).at(element_offset) << " ";
					}
					cout << " has error: " << centroid_error << " and contains " << num_assignments << endl;
				}
			}
			if(verbose){
				cout << "\tFinal error: " << error_sum_this_round << endl;
			}
		}
		
		// Leave only the absolute-max-centroid number of biggest matches
		if(absolute_max_centroids > 0){
			sort(centroid_sizes.begin(), centroid_sizes.end(), sort_centroid_members);
		
			for(int i = absolute_max_centroids; i < centroid_sizes.size(); i++){
				centroids->erase(centroids->begin() + i);
			}
		}
	}

	void get_2d_coord_for_3d_depth_coord(double *x_2d, double *y_2d, pcl::PointCloud<pcl::PointXYZRGB> *cloud, vector<double> *point_3d){
		// terrible method for finding which 2d coord on the image plain maps to the 3d point in the depth data.
		// Should instead figure out the projection matrix onto the image plain, but I can't find that.
		// It looks for the closest point based on euclidean distance
		
		double desired_point[3] = {point_3d->at(0), point_3d->at(1), point_3d->at(2)};
		//cout << "desired_point: " << desired_point[0] << "," << desired_point[1] << "," << desired_point[2] << endl;
		
		double xyz[3];
		double best_xyz[3];
		double best_distance = 99999999;
		double this_distance;
		for(int x = 0; x < cloud->width; x++){
			for(int y = 0; y < cloud->height; y++){
				get_xyz_from_xyzrgb(x, y, cloud, xyz);
				if(!is_valid_xyz(xyz)){
					continue;
				}
				
				this_distance = euclid_distance_3d_not_vec_double(xyz, desired_point);
				//cout << "x: " << x << ", y: " << y << ", xyz: " << xyz[0] << "," << xyz[1] << "," << xyz[2] << ", distance: " << this_distance << endl;
			
				if(this_distance < best_distance){
					//cout << "updating " << this_distance << " < " << best_distance << ", " << x << "x" << y << endl;
					(*x_2d) = x;
					(*y_2d) = y;
					best_distance = this_distance;
					memcpy(best_xyz, xyz, 3 * sizeof(double));
				}
			}
		}
	}
	
	void perform_frame_combinations(vector< vector<double> > *combined, vector< vector<double> > *matches, vector< vector< vector<double> > > *previous_frames){
		combined->clear();
		// load this round into combined list
		for(int i = 0; i < matches->size(); i++){
			combined->push_back(matches->at(i));
		}
		// load points from previous rounds into combined list
		for(int i = 0; i < args->additional_color_match_frames_to_combine && i < previous_frames->size(); i++){
			// load points from specific frame
			for(int j = 0; j < previous_frames->at(i).size(); j++){
				combined->push_back(previous_frames->at(i).at(j));
			}
		}
		if(args->additional_color_match_frames_to_combine > 0){
			// update previous-frames to include this one. higher = newer, lower = older.

			// if we're maxed out or over the limit (if it changes), delete the last one.
			while(previous_frames->size() >= args->additional_color_match_frames_to_combine){
				previous_frames->erase(previous_frames->begin());
			}
			// add new frame's data to last slot
			int new_frame_index = previous_frames->size();
			
			previous_frames->push_back(  vector< vector<double> >(0) );
		
			for(int i = 0; i < matches->size(); i++){
				previous_frames->at(new_frame_index).push_back(matches->at(i));
			}
		}
	}

	void color_pixels(cv::Mat *im_matrix, vector< vector<double> > *points, struct rgb *color){
		int x,y;
		for(int i = 0; i < points->size(); i++){
			x = points->at(i).at(0);
			y = points->at(i).at(1);
			im_matrix->at<cv::Vec3b>(y,x)[0] = color->b;
			im_matrix->at<cv::Vec3b>(y,x)[1] = color->g;
			im_matrix->at<cv::Vec3b>(y,x)[2] = color->r;
		}
	}

	//void cloudCb (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& input){
	void cloudCb (const sensor_msgs::PointCloud2ConstPtr& input){
		if(args->terminate){
			cv::destroyWindow(OPENCV_WINDOW);
			return;
		}
		
		bool verbose = args->verbose;
		
		// skip every-other frame for faster rendering
		if(++frames % 2 == 0){
			//cout << "skipping " << frames << endl;
			return;
		}
		
		
		pcl::PointCloud<pcl::PointXYZRGB> cloud;
  		pcl::fromROSMsg (*input, cloud);
		
		int i, j, x, y;

		cv::Mat im_matrix(cloud.height, cloud.width, CV_8UC3);
		
		// store 2d matches and 3d matches
		vector< vector<double> > object_matched_points_2d;
		vector< vector<double> > object_matched_points_3d;


		vector< vector<double> > jaco_tag_matched_points_2d;
		vector< vector<double> > jaco_tag_matched_points_3d;
		

		bool match;
		for (int y = 0; y < im_matrix.rows; y++) {
			for (int x = 0; x < im_matrix.cols; x++) {
				
				// find green_cylinder
				match = find_match_by_color(&im_matrix, &cloud, x, y, &object_matched_points_2d, &object_matched_points_3d, &green_cylinder, verbose);

				// find jaco tag
				match |= find_match_by_color(&im_matrix, &cloud, x, y, &jaco_tag_matched_points_2d, &jaco_tag_matched_points_3d, &blue_tag, verbose);
				
			}
		}

		
		vector< vector<double> > object_matched_points_2d_combined;
		vector< vector<double> > object_matched_points_3d_combined;

		perform_frame_combinations(&object_matched_points_2d_combined, &object_matched_points_2d, &object_matched_points_2d_previous_rounds);
		perform_frame_combinations(&object_matched_points_3d_combined, &object_matched_points_3d, &object_matched_points_3d_previous_rounds);
		

		vector< vector<double> > jaco_tag_matched_points_2d_combined;
		vector< vector<double> > jaco_tag_matched_points_3d_combined;

		perform_frame_combinations(&jaco_tag_matched_points_2d_combined, &jaco_tag_matched_points_2d, &jaco_tag_matched_points_2d_previous_rounds);
		perform_frame_combinations(&jaco_tag_matched_points_3d_combined, &jaco_tag_matched_points_3d, &jaco_tag_matched_points_3d_previous_rounds);
		

		// draw pixels on screen
		if(args->draw_pixel_match_color){
			color_pixels(&im_matrix, &object_matched_points_2d_combined, &match_color);

			color_pixels(&im_matrix, &jaco_tag_matched_points_2d_combined, &jaco_match_color);
		}
		
		/*
		cout << "2d Matched: " << object_matched_points_2d.size() << ", combined: " << object_matched_points_2d_combined.size() << endl;
		cout << "3d Matched: " << object_matched_points_3d.size() << ", combined: " << object_matched_points_3d_combined.size() << endl;
		cout << "new last round: " << object_matched_points_3d_combined.size() << ", current: " << object_matched_points_3d.size() << endl;
		*/
		if(verbose)
			cout << "post: " << object_matched_points_2d.size() << endl;
		/*		
		if(frames > 1){
			return;
		}
		*/

		//pthread_mutex_lock(&centroid_mutex);
		// Arbitrary 5 initially, maybe we know we're looking for n of whatever.
		int max_centroids_to_try = 5;
		kmeans_cluster_and_centroid(&object_matched_points_3d_combined, &object_centroids_3d, max_centroids_to_try, args->num_objects_in_scene, verbose);
		
		kmeans_cluster_and_centroid(&jaco_tag_matched_points_3d_combined, &jaco_tag_centroids_3d, max_centroids_to_try, args->num_jaco_arms_in_scene, verbose);
		
		//compute_centroids(&object_matched_points_2d, &object_matched_points_3d, &object_centroids_2d, &object_centroids_3d, verbose);
		//compute_centroids(&jaco_tag_matched_points_2d, &jaco_tag_matched_points_3d, &jaco_tag_centroids_2d, &jaco_tag_centroids_3d, verbose);

		
		//pthread_mutex_unlock(&centroid_mutex);
		
		//cout << "Num centroids: " <<  object_centroids_3d.size() << ", " << im_matrix.rows << " by " << im_matrix.cols << endl;
		double x_2d,y_2d;
		for(i = 0; i < object_centroids_3d.size(); i++){
			// http://stackoverflow.com/questions/6139451/how-can-i-convert-3d-space-coordinates-to-2d-space-coordinates
			get_2d_coord_for_3d_depth_coord(&x_2d, &y_2d, &cloud, &object_centroids_3d.at(i));

			// Draw an circle on the video stream around the 2d centroids
			cv::circle(im_matrix, cv::Point(x_2d, y_2d), 20, CV_RGB(match_color.r,match_color.g,match_color.b));
		}
		
		for(i = 0; i < jaco_tag_centroids_3d.size(); i++){
			get_2d_coord_for_3d_depth_coord(&x_2d, &y_2d, &cloud, &jaco_tag_centroids_3d.at(i));
			
			// Draw an circle on the video stream around the 2d centroids
			cv::circle(im_matrix, cv::Point(x_2d, y_2d), 20, CV_RGB(jaco_match_color.r,jaco_match_color.g,jaco_match_color.b));
			
		}
		
		if(object_centroids_3d.size() == 2){
			double c0_xyz[3];
			double c1_xyz[3];
			
			c0_xyz[0] = object_centroids_3d.at(0).at(0);
			c0_xyz[1] = object_centroids_3d.at(0).at(1);
			c0_xyz[2] = object_centroids_3d.at(0).at(2);
			if(verbose)
				cout << "\tdistance between centroids: " << euclid_distance_3d_not_vec(c0_xyz, c1_xyz) << endl;
		}
		
		int num_object_centroids_3d = object_centroids_3d.size();
		if(num_object_centroids_3d > 0){
			if(args->num_objects != num_object_centroids_3d){
				struct xyz *temp = (struct xyz *) malloc (num_object_centroids_3d * sizeof(struct xyz));
				if(args->object_xyz){
					free(args->object_xyz);
				}
				args->object_xyz = temp;


				double *temp_dist = (double *) malloc (num_object_centroids_3d * sizeof(double));
				if(args->object_distances){
					free(args->object_distances);
				}
				args->object_distances = temp_dist;
			}
			args->num_objects = num_object_centroids_3d;
			for(i = 0; i < num_object_centroids_3d; i++){
				
				args->object_xyz[i].x = object_centroids_3d[i].at(0);
				args->object_xyz[i].y = object_centroids_3d[i].at(1);
				args->object_xyz[i].z = object_centroids_3d[i].at(2);
				
				args->object_distances[i] = vector_length_3d_struct(& args->object_xyz[i]);
				
				if(verbose){
					cout << "Distance to Object (" << i << ") : " << args->object_distances[i] << endl;
				}



				
			}
		}
		
		
		int num_jaco_tag_centroids_3d = jaco_tag_centroids_3d.size();
		if(num_jaco_tag_centroids_3d > 0){
			if(args->num_jaco_tags != num_jaco_tag_centroids_3d){
				struct xyz *temp = (struct xyz *) malloc (num_jaco_tag_centroids_3d * sizeof(struct xyz));
				if(args->jaco_tag_xyz){
					free(args->jaco_tag_xyz);
				}
				args->jaco_tag_xyz = temp;


				double *temp_dist = (double *) malloc (num_jaco_tag_centroids_3d * sizeof(double));
				if(args->jaco_distances){
					free(args->jaco_distances);
				}
				args->jaco_distances = temp_dist;
			}
			args->num_jaco_tags = num_jaco_tag_centroids_3d;
			for(i = 0; i < num_jaco_tag_centroids_3d; i++){
				
				args->jaco_tag_xyz[i].x = jaco_tag_centroids_3d[i].at(0);
				args->jaco_tag_xyz[i].y = jaco_tag_centroids_3d[i].at(1);
				args->jaco_tag_xyz[i].z = jaco_tag_centroids_3d[i].at(2);
				
				args->jaco_distances[i] = vector_length_3d_struct(& args->jaco_tag_xyz[i]);
				
				if(verbose){
					cout << "Distance to JACO tag (" << i << ") : " << args->jaco_distances[i] << endl;
				}



				
			}
		}
		
		// Update GUI Window
		
		cv::imshow(OPENCV_WINDOW, im_matrix);
		cv::waitKey(3);
	}
	
	
	void compute_centroids(vector< vector<double> > *matches_2d, vector< vector<double> > *matches_3d, vector< vector<double> > *centroids_2d, vector< vector<double> > *centroids_3d, bool verbose){
		// get all distances between points
		if(verbose)
			cout << "beginning centroids" << endl;
		vector<double> distances;
		int i, j, k;
		double dist;
		int num_matches = matches_3d->size();
		for(i = 0; i < num_matches; i++){
			for(j = i+1; j < num_matches; j++){
				dist = euclid_distance_3d(matches_3d->at(i), matches_3d->at(j));
				if(std::isnan(dist)){
					//cout << "distance: " << dist << endl;
				}else{
					//cout << "distance: " << dist << endl;
					distances.push_back(dist);
				}
			}
		}
		if(verbose){
			cout << "finished distances" << cout;
		}
		// do sort/unique on distance measures
		sort (distances.begin(), distances.end());
		std::vector<double>::iterator it;
		it = std::unique (distances.begin(), distances.end());
		distances.resize( std::distance(distances.begin(),it) );
		
		/*
		FILE *fp = fopen("distances.log", "w");
		for(i = 0; i < distances.size(); i++){
			fprintf(fp, "%f\n", distances.at(i));
		}
		fclose(fp);
		*/
		// now distances are unique
		if(verbose)
			cout << "\tFinal distances: " << distances.size() << endl;
	
		// find clusters based on histogram
		double missing = 0, index = 0, current;
		double missing_needed = 0.1;
		double last = -1;
		double delta, threshold;
		while(index < distances.size() && missing < missing_needed){
			if(-1 == last){
				last = distances.at(index);
				index++;
				continue;
			}
		
			current = distances.at(index);
			missing = current - last; // positive
			last = current;
			index++;
		}
		threshold = current;
		
		if(verbose)
			cout << "\tCutoff: " << missing << " @ " << threshold << ","<< last << "," << index << endl;
		
		
		bool found_cluster, matched_all;
		
		vector< vector< vector<double> > > clusters_2d;
		vector< vector< vector<double> > > clusters_3d;
		
		if(verbose)
			cout << "beginning clustering" << endl << endl;
		for(i = 0; i < num_matches; i++){
			// find which cluster the point belongs in, or create new cluster
			found_cluster = false;
		
			for(j = 0; !found_cluster && j < clusters_3d.size(); j++){
				matched_all = true;
				for(k = 0; matched_all && k < clusters_3d.at(j).size(); k++){
					dist = euclid_distance_3d(matches_3d->at(i), clusters_3d.at(j).at(k));
					if(dist > threshold){
						matched_all = false;
					}
				}
			
				if(matched_all && clusters_3d.size() > 0){
					clusters_2d.at(j).push_back(matches_2d->at(i));
					clusters_3d.at(j).push_back(matches_3d->at(i));
					found_cluster = true;
				}
			}
			if(!found_cluster){
				// create new cluster
				clusters_2d.push_back( vector< vector<double> >(0) );
				clusters_3d.push_back( vector< vector<double> >(0) );
				// add this point to the cluster we just made
				clusters_2d.at(clusters_2d.size() - 1).push_back( matches_2d->at(i));
				clusters_3d.at(clusters_3d.size() - 1).push_back( matches_3d->at(i));
			}
		}
	
		if(verbose){
			cout << "done clustering" << endl << endl;
			cout << "\tNum clusters: " << clusters_3d.size() << endl << endl;
		}
		int minimum_points_per_cluster = 100;
		
		centroids_2d->clear();
		centroids_3d->clear();
	
		double x_sum, y_sum, z_sum, num_centroids, total;
		double x_sum_2d, y_sum_2d;
		for(i = 0; i < clusters_3d.size(); i++){
			total = clusters_3d.at(i).size();
			if(verbose){
				cout << "Cluster " << i << " has 3d: " << clusters_3d.at(i).size() << ", 2d: " << clusters_2d.at(i).size() << endl;
			}
			if(total < minimum_points_per_cluster){
				continue;
			}
		
			x_sum = y_sum = z_sum = 0.0;
			x_sum_2d = y_sum_2d = 0;
			for(j = 0; j < total; j++){
				
				//cout << "++ " << clusters_2d.at(i).at(j).at(0) << ","<< clusters_2d.at(i).at(j).at(1) << "," << clusters_3d.at(i).at(j).at(0) << "," << clusters_3d.at(i).at(j).at(1) << ","<< clusters_3d.at(i).at(j).at(2) << "," << endl;
				x_sum += clusters_3d.at(i).at(j).at(0);
				y_sum += clusters_3d.at(i).at(j).at(1);
				z_sum += clusters_3d.at(i).at(j).at(2);

				x_sum_2d += clusters_2d.at(i).at(j).at(0);
				y_sum_2d += clusters_2d.at(i).at(j).at(1);
			}
			
			//cout << "<< " << x_sum_2d << "," << y_sum_2d << ","<< x_sum << ","<< y_sum << ","<< z_sum << endl;
			x_sum_2d = (x_sum_2d / total);
			y_sum_2d = (y_sum_2d / total);

			
			x_sum = (x_sum / total);
			y_sum = (y_sum / total);
			z_sum = (z_sum / total);
			//cout << ">> " << x_sum_2d << "," << y_sum_2d << ","<< x_sum << ","<< y_sum << ","<< z_sum << endl;

			num_centroids = centroids_2d->size();
			centroids_2d->push_back(vector<double>(2));
			centroids_2d->at(num_centroids).at(0) = (double) x_sum_2d;
			centroids_2d->at(num_centroids).at(1) = (double) y_sum_2d;

			centroids_3d->push_back(vector<double>(3));
			centroids_3d->at(num_centroids).at(0) = (double) x_sum;
			centroids_3d->at(num_centroids).at(1) = (double) y_sum;
			centroids_3d->at(num_centroids).at(2) = (double) z_sum;
			
			if(verbose){
				cout << "3d centroid computed at: " << centroids_3d->at(num_centroids).at(0) << "," << centroids_3d->at(num_centroids).at(1) << "," << centroids_3d->at(num_centroids).at(2) << endl;
				cout << "2d centroid computed at: " << centroids_2d->at(num_centroids).at(0) << "," << centroids_2d->at(num_centroids).at(1) << endl;
			}
		}
		
		if(verbose){
			cout << "\tNum centroids: " << centroids_3d->size() << endl;
			for(i = 0; i < centroids_3d->size(); i++){
				cout << "centroid " << i << endl;
				cout << "\t" << centroids_3d->at(i).at(0) << "," << centroids_3d->at(i).at(1) << "," << centroids_3d->at(i).at(2) << endl;
			}
		}	
	}
	
};


void *handle_viz(void *thread_args){
	struct viz_thread_args *viz_args = (struct viz_thread_args *) thread_args;
	
	ros::init(*viz_args->argc, *viz_args->argv, "image_converter");
	ImageConverter ic;
	ic.set_args(viz_args);
	ros::Rate r(10);
	
	while(!viz_args->terminate){
		ros::spinOnce();
		r.sleep();
	}
	cout << endl << "Terminating viz" << endl;
	cv::destroyWindow(OPENCV_WINDOW);
}




#endif

