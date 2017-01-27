#ifndef _VIZH_
#define _VIZH_

#include <iostream>

#include <mlpack/methods/kmeans/kmeans.hpp>

#include <vector>
#include <pthread.h>
#include <cmath>
#include <cfloat>
#include <unordered_map>
#include <queue>

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


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "dbscan.h"

#define ARM_SKELETON_POINT_FILE "jaco_skeleton.csv"
#define ARM_PCL_POINT_FILE "jaco_pcl.csv"

#define VIZ_DO_NOOPS false


using namespace mlpack::kmeans;

// orange color: #ef5e25
struct rgb_set orange = {1, {{239, 94, 37}} };

// blue tag color: #2499bc, #60fffe, #3fcde3
struct rgb_set blue_tag = {3, {{64, 166, 189}, {96,255,254}, {71,167,191}} };

// bottle color: #45c5d2
struct rgb_set bottle = {1, {{69, 197, 210}} };

// green cylinder color: #4cab6c
struct rgb_set green_cylinder = {1, {{76,171,108}, } };

// orange cylinder color: #d56438
struct rgb_set orange_bottle_cylinder = {1, {{213,100,56} } };

// pixel shading color for matches
struct rgb match_color = {0xff, 0xd7, 0x00};

// table color
struct rgb table_color = {0xef, 0x28, 0xd1};

// wall color
struct rgb wall_color = {0x77, 0x89, 0xff};

// misc color
struct rgb misc_color = {0xff, 0xd8, 0xb5};

// white
struct rgb white_color = {0xff, 0xff, 0xff};

// pixel shading color for jaco tag, red
struct rgb jaco_match_color = {0xff, 0, 0};

// pixel shading color for jaco arm, blue
struct rgb jaco_arm_match_color = {0, 0, 0xff};

// pixel shading color for trying to find jaco arm, lightblue
struct rgb jaco_arm_tried_color = {0xAD, 0xD8, 0xE6};

struct big_rgb_set big_set_colors = {24, {
	{2,63,165},{125,135,185},{190,193,212},{214,188,192},{187,119,132},{142,6,59},{74,111,227},{133,149,225},{181,187,227},{230,175,185},{224,123,145},{211,63,106},{17,198,56},{141,213,147},{198,222,199},{234,211,198},{240,185,141},{239,151,8},{15,207,192},{156,222,214},{213,234,231},{243,225,235},{246,196,225},{247,156,212}
}
};


vector<short> get_hex_color(int n){
	vector<short> color;
	struct rgb *chosen = &big_set_colors.colors[n % big_set_colors.num_colors];
	color.push_back(chosen->r);
	color.push_back(chosen->g);
	color.push_back(chosen->b);
	return color;
}
void get_unique_colors(int n, vector<vector<short>> *colors){
	for(int i = 0; i < n; i++){
		colors->push_back(get_hex_color(i));
	}
}

void *pcl_viz(void *thread_args){
	struct pcl_viz_args *pcl_viz_args = (struct pcl_viz_args *) thread_args;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr currently_visualizing;
	
	// copied/tweaked from http://pointclouds.org/documentation/tutorials/random_sample_consensus.php
	// --------------------------------------------
	// -----Open 3D viewer and add point cloud-----
	// --------------------------------------------
	bool ever_drawn_viewer = false;
	
	//pcl::visualization::Camera cam;
	
	pcl::PointCloud<pcl::PointXYZRGB> temp;
	while(!pcl_viz_args->terminated && !viewer->wasStopped()){
		while(!(*(pcl_viz_args->pcl_viz_input_ready)) && !viewer->wasStopped()){
			boost::this_thread::sleep (boost::posix_time::microseconds (100000));
		}
		
		temp.clear();
		for(int i = 0; i < pcl_viz_args->pcl_viz_cloud_input->size(); i++){
			temp.push_back(pcl_viz_args->pcl_viz_cloud_input->at(i));
		}
		currently_visualizing = temp.makeShared();
		(*(pcl_viz_args->pcl_viz_input_ready)) = false;
		
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb (currently_visualizing);
		
		if(!ever_drawn_viewer){
			viewer->setBackgroundColor(0, 0, 0);
			
			viewer->addPointCloud<pcl::PointXYZRGB> (currently_visualizing, rgb, "sample cloud");
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
			
			boost::this_thread::sleep (boost::posix_time::microseconds (10000000));
		}else{
			viewer->updatePointCloud<pcl::PointXYZRGB> (currently_visualizing, rgb, "sample cloud");
		}
		if(!ever_drawn_viewer || (*pcl_viz_args->reset_camera)){
			viewer->setCameraPosition(-0.680567,0.0879865,-1.29801,0.552615,-0.16985,1.571,-0.0574351,-0.996241,-0.0648442);
			
			ever_drawn_viewer = true;
			(*pcl_viz_args->reset_camera) = false;
		}
		//viewer->getCameraParameters(cam);
		
		//cout << "Cam settings: setCameraPosition(" << cam.pos[0] << "," << cam.pos[1] << "," << cam.pos[2] << "," << cam.focal[0] << "," << cam.focal[1] << "," << cam.focal[2] << "," << cam.view[0] << "," << cam.view[1] << "," << cam.view[2] << ")" << endl;
		
		while (!(*(pcl_viz_args->pcl_viz_input_ready)) && !viewer->wasStopped()){
			viewer->spinOnce (100);
			boost::this_thread::sleep (boost::posix_time::microseconds (100000));
		}
	}
	
}

void pcl_viz_this_cloud(bool *is_ready, pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZRGB> *input, vector<vector<short>> *colors){
	if(!(*is_ready)){
		if(colors->size() == source->size()){
			int i;
			input->clear();
			pcl::PointXYZRGB point;
		
			for(i = 0; i < source->size(); i++){
				point.x = source->at(i).x;
				point.y = source->at(i).y;
				point.z = source->at(i).z;
			
				point.r = colors->at(i).at(0);
				point.g = colors->at(i).at(1);
				point.b = colors->at(i).at(2);
				input->push_back(point);
			}
		}else{
			cout << "invalid # of colors" << endl;
		}
		(*(is_ready)) = true;
		
	}
}

void load_white(int n, vector<vector <short>> *colors){
	vector<short> white;
	white.push_back(0xff);
	white.push_back(0xff);
	white.push_back(0xff);
	for(int i = 0; i < n; i++){
		colors->push_back(white);
	}
}

void pcl_viz_this_vector_of_points(bool *is_ready, vector< vector<double> > *source, pcl::PointCloud<pcl::PointXYZRGB> *input, vector<vector<short>> *colors){
	pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointXYZ point;
	for(int i = 0; i < source->size(); i++){
		point.x = source->at(i).at(0);
		point.y = source->at(i).at(1);
		point.z = source->at(i).at(2);
		new_cloud->push_back( point );
	}
	pcl_viz_this_cloud(is_ready, new_cloud, input, colors);
}


struct find_arm_args{
	bool verbose;
	vector< vector<double> > *map_2d_combined;
	vector< vector<double> > *jaco_tag_matched_points_3d_combined;
	vector< vector<double> > *all_3d_points_combined;
	vector< vector< vector<double> > > *arm_clusters_2d_points;
	vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > *arm_clusters_3d_points;
	int *validated_cluster;
	vector< vector<double> > *table_points;
	vector< vector<double> > *wall_points;
	vector< vector<double> > *non_table_or_wall_points;
	bool use_dbscan;
	
	arma::Row<size_t> *arm_skeleton_assignments;
	vector< vector<double> > *arm_skeleton_centroids;
	double *cluster_error_cutoff;
	
	bool *pcl_viz_input_ready;
	pcl::PointCloud<pcl::PointXYZRGB> *pcl_viz_cloud_input;
	pcl_vizualizations viz_selection;
	
	int *save_jaco_skeleton_frames;
	
	bool *show_viz_cluster_colors;
};

struct temporal_smoothing_args{
	vector< vector<double> > *combined;
	vector< vector<double> > *matches;
	vector< vector< vector<double> > > *previous_frames;
	int *additional_color_match_frames_to_combine;
};

bool colors_are_similar(struct rgb *x, struct rgb *y){
	double distance = sqrt( (x->r - y->r)*(x->r - y->r) + (x->g - y->g)*(x->g - y->g) + (x->b - y->b)*(x->b - y->b) );
	// epsilon subject to change here, was trial and error early on.
	return distance < 50;
}


double degrees_between_3d_vectors(double x1, double y1, double z1, double x2, double y2, double z2){
	// http://stackoverflow.com/questions/14066933/direct-way-of-computing-clockwise-angle-between-2-vectors
	double dot = x1*x2 + y1*y2 + z1*z2;
	double lenSq1 = x1*x1 + y1*y1 + z1*z1;
	double lenSq2 = x2*x2 + y2*y2 + z2*z2;
	double radians = acos(dot/sqrt(lenSq1 * lenSq2));
	double degrees = radians * 57.2958; // (180 / M_PI) = 57.2958
	return degrees;
}


double degrees_between_2d_vectors(double x1, double y1, double x2, double y2){
	// http://stackoverflow.com/questions/14066933/direct-way-of-computing-clockwise-angle-between-2-vectors
	double dot = x1*x2 + y1*y2;      // dot product
	double det = x1*y2 - y1*x2;      // determinant
	double radians = atan2(det, dot);  // atan2(y, x) or atan2(sin, cos)
	double degrees = radians * 57.2958; // (180 / M_PI) = 57.2958
	return degrees;
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

double vector_length_2d(double *xy){
	return (double) sqrtf(xy[0]*xy[0] + xy[1]*xy[1]);
}

double vector_length_3d_struct(struct xyz *xyz){
	return (double) sqrtf(xyz->x*xyz->x + xyz->y*xyz->y + xyz->z*xyz->z);
}

int color_normalize(double dist, double max, int span){
	return (int)((dist / max) * span);
}

void get_xyz_from_xyzrgb(int x, int y, pcl::PointCloud<pcl::PointXYZRGB> *cloud, double *xyz){
	if(x >= cloud->width || x < 0 || y >= cloud->height || y < 0){
		xyz[0] = NAN;
		xyz[1] = NAN;
		xyz[2] = NAN;
		return;
	}
	
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

bool is_visible_angle(double x2, double y2, double z2, double angle){
	return degrees_between_3d_vectors(0,0,1, x2,y2,z2) <= angle;
}

bool is_valid_xyz(double *xyz){
	return !std::isnan(xyz[0]) && !std::isnan(xyz[1]) && !std::isnan(xyz[2]);
}

bool do_pixel_test(int x, int y, cv::Mat *image, struct rgb *desired, vector< vector<double> > *matches_2d, vector< vector<double> > *matches_3d, pcl::PointCloud<pcl::PointXYZRGB> *cloud, double max_distance, double min_distance, double visible_angle){
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
		double v_length = vector_length_3d(xyz);
		if(v_length < min_distance || max_distance < v_length){
			return false;
		}

		if(!is_visible_angle(xyz[0], xyz[1], xyz[2], visible_angle)){
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

int get_key_from_coordinate(vector< double > xy, int rows){
	return xy.at(0) + xy.at(1) * rows;
}

void attempt_plane_segmentation(int valid_surface_size, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, vector<vector<double>> *orig_2d, vector<vector<double>> *match_2d, pcl::PointCloud<pcl::PointXYZ>::Ptr non_match_3d, vector< vector<double> > *non_match_2d){
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object, get the table
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.03);
	seg.setInputCloud(input_cloud);
	seg.segment(*inliers, *coefficients);
	
	unordered_map<int, bool> valid_indices;
	int i = 0;
	if(inliers->indices.size() > valid_surface_size){
		for(i = 0; i < inliers->indices.size(); i++){
			match_2d->push_back( orig_2d->at(inliers->indices[i]) );
			// add valid indices to lookup table
			valid_indices[inliers->indices[i]] = true;
		}
		
	}
	for(i = 0; i < orig_2d->size(); i++){
		if(valid_indices.find(i) == valid_indices.end()){
			// if not in lookup table, they aren't part of the table
			non_match_3d->push_back(input_cloud->at(i));
			non_match_2d->push_back(orig_2d->at(i));
		}
	}
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

void build_centroids(vector< vector<double> > *centroids, arma::mat *k_centroids, int num_elements){
	int num_centroids = centroids->size();
	centroids->push_back(vector<double>(num_elements));
	for(int element_offset = 0; element_offset < num_elements; element_offset++){
		centroids->at(num_centroids).at(element_offset) = (*k_centroids)[num_centroids * num_elements + element_offset];
	}
}

bool too_many_clusters(double new_val, double old_val, double *error_cutoff){
	// TODO: figure out a good way to return true on elbow function
	// https://en.wikipedia.org/wiki/Determining_the_number_of_clusters_in_a_data_set
	
	// difference in readings must be less than 10% of the previous value.
	//cout << old_val << "   " << new_val << "   " << (old_val - new_val) << "   " << (0.1 * old_val) << "   " << ((old_val - new_val) < 0.1 * old_val ? "break" : "continue") << endl;
	return old_val > new_val && (old_val - new_val) < (*error_cutoff) * old_val;
}

// Function sorts vectors with (centroid id, number of members) pairs
bool sort_centroid_members(vector< double > a, vector< double > b) { return a[1] > b[1]; }

void kmeans_cluster_and_centroid(vector< vector<double> > *samples, vector< vector<double> > *centroids, int max_centroids_to_try, int absolute_max_centroids, arma::Row<size_t> *assignments, double *error_cutoff, bool verbose){
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
	//cout << "Data rows: " << data.n_rows << ", cols: " << data.n_cols << " from " << samples->size() << " points" << endl;
	
	
	
	// The number of clusters we are getting.
	
	// The centroids will be stored in this matrix.
	arma::mat k_centroids;
	// Initialize with the default arguments.
	KMeans<> k(1000);
	
	double centroid_error;
	int num_assignments;
	vector< vector< double > > centroid_sizes;

	int ideal_centroid_count = -1;
	double error_sum_this_round, error_sum_last_round;
	vector<double> avg_error_history;
	int avg_error_history_size = avg_error_history.size();
	for(int num_centroids = 1; num_centroids < max_centroids_to_try; num_centroids++){
		centroids->clear();
		assignments->reset();
		k_centroids.reset();
		
		if(samples->size() < num_centroids){
			//cout << "Skipping due to " << samples->size() << " < " << num_centroids << endl;
			continue;
		}
		try{
			k.Cluster(data, num_centroids, *assignments, k_centroids);
		}catch(...){
			//cout << "caught error during kmeans cluster" << endl;
		}
		
		error_sum_this_round = 0;
		
		centroid_sizes.clear();
		for(int this_centroid = 0; this_centroid < num_centroids; this_centroid++){
			build_centroids(centroids, &k_centroids, data.n_rows);

			centroid_error = compute_centroid_error(&(centroids->at(this_centroid)), this_centroid, assignments, samples, &num_assignments);
			centroid_sizes.push_back( vector< double >(2) );
			centroid_sizes.at(this_centroid).at(0) = this_centroid;
			centroid_sizes.at(this_centroid).at(1) = num_assignments;

			error_sum_this_round += centroid_error;

			if(verbose){
				cout << "(" << num_centroids << ") centroid " << this_centroid << ": ";
				for(int element_offset = 0; element_offset < data.n_rows; element_offset++){
					cout << centroids->at(this_centroid).at(element_offset) << " ";
				}
				cout << " has error: " << centroid_error << " and contains " << num_assignments << " samples" << " samples (running error: " << error_sum_this_round << endl;
			}
			

		}
		avg_error_history.push_back(error_sum_this_round / num_centroids);
		avg_error_history_size = avg_error_history.size();
		if(verbose){
			for(int err = 0; err < avg_error_history.size(); err++){
				cout << (err+1) << "," << avg_error_history.at(err) << endl;
			}
		}
		if(avg_error_history.size() > 1){
			// Have to try at least 1 centroid before we could break for elbow, since not enough comparisons
			//cout << "trying " << avg_error_history.at(avg_error_history_size - 1) << " vs " << avg_error_history.at(avg_error_history_size - 2) << endl;
			if(too_many_clusters(avg_error_history.at(avg_error_history_size - 1), avg_error_history.at(avg_error_history_size - 2), error_cutoff)){
				ideal_centroid_count = num_centroids+1;
				//cout << "FOUND ENOUGH " << ideal_centroid_count << endl;
				break;
			}
		}
		error_sum_last_round = error_sum_this_round;
	}
	if(ideal_centroid_count == -1){
		ideal_centroid_count = max_centroids_to_try;
	}
	
	
	// Leave only the absolute-max-centroid number of biggest matches
	if(absolute_max_centroids > 0){
		// Messes up mappings with assignments, though, so let's clear these out.
		assignments->reset();
		sort(centroid_sizes.begin(), centroid_sizes.end(), sort_centroid_members);
	
		for(int i = absolute_max_centroids; i < centroid_sizes.size(); i++){
			centroids->erase(centroids->begin() + i);
		}
	}
}

void *do_find_arm(void *thread_args){
	struct find_arm_args *args = (struct find_arm_args *) thread_args;

	if(args->verbose){
		cout << "find arm start" << endl;
	}
	
	if(VIZ_DO_NOOPS){
		//cout << "skipping" << endl;
		return NULL;
	}
	
	int i = 0, j = 0;

	vector< vector<double> > non_table_points;
	//create cloud
	pcl::PointXYZ temp_point;
	pcl::PointCloud<pcl::PointXYZ>::Ptr all_3d_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr non_table_3d_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr non_table_or_wall_3d_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	
	bool found_jaco_tag = args->jaco_tag_matched_points_3d_combined->size() > 0;
	vector<double> sample_jaco_point;
	
	
	int sample_jaco_point_index = -1;
	if(found_jaco_tag){
		sample_jaco_point = args->jaco_tag_matched_points_3d_combined->at(0);
	}
	for(i = 0; i < args->all_3d_points_combined->size(); i++){
		temp_point.x = args->all_3d_points_combined->at(i)[0];
		temp_point.y = args->all_3d_points_combined->at(i)[1];
		temp_point.z = args->all_3d_points_combined->at(i)[2];

		if(found_jaco_tag && sample_jaco_point_index < 0 && temp_point.x == sample_jaco_point[0] && temp_point.y == sample_jaco_point[1] && temp_point.z == sample_jaco_point[2]){
			sample_jaco_point_index = i;
		}

		all_3d_cloud->push_back(temp_point);
	}
	if(args->viz_selection == PCL_ALL){
		vector<vector<short>> colors;
		load_white(all_3d_cloud->size(), &colors);
		pcl_viz_this_cloud(args->pcl_viz_input_ready, all_3d_cloud, args->pcl_viz_cloud_input, &colors);
	}
	if(found_jaco_tag){
		
		int valid_surface_size = 50000;
	
		// remove one surface (table?)
		attempt_plane_segmentation(valid_surface_size, all_3d_cloud, args->map_2d_combined, args->table_points, non_table_3d_cloud, &non_table_points);
		
		if(args->table_points->size() >= valid_surface_size){
			// remove another surface (wall?)
			attempt_plane_segmentation(valid_surface_size, non_table_3d_cloud, &non_table_points, args->wall_points, non_table_or_wall_3d_cloud, args->non_table_or_wall_points);
		}
		//cout << "start blob" << endl;


		double eps = 0.02; // 2cm
		if(args->use_dbscan){
			int min_points = 100; // 3cm
			clustering::DBSCAN dbscan( eps, min_points );
			ublas::matrix< double > input_points(non_table_or_wall_3d_cloud->size(), 3);
			//cout << "Attempting [" << input_points.size1() << "," << input_points.size2() << "] input, jaco sample: " << sample_jaco_point_index << endl;
			pcl::PointXYZ point;
			for(i = 0; i < non_table_or_wall_3d_cloud->size(); i++){
				point = non_table_or_wall_3d_cloud->at(i);
				input_points(i, 0) = point.x;
				input_points(i, 1) = point.y;
				input_points(i, 2) = point.z;
			}
	
			dbscan.fit( input_points );
			std::vector< int32_t > labels = dbscan.get_labels();
			int num_labels = labels.size();
			int jaco_tag_cluster_id = (sample_jaco_point_index >= 0 ? dbscan.get_labels().at(sample_jaco_point_index) : -9999);
		
			//cout << "found " << num_labels << " labels, with jaco tag at cluster (" << sample_jaco_point_index << ") " << jaco_tag_cluster_id << endl;
		
			vector< vector<double> > cluster_points;
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
			
			for(i = 0; i < num_labels; i++){
				if(labels.at(i) != jaco_tag_cluster_id){
					continue;
				}
				cloud_cluster->points.push_back(non_table_or_wall_3d_cloud->points[i]);
			
				cluster_points.push_back(args->non_table_or_wall_points->at( i ));
				
				//cout << "Pushing 2d " << index << ", " << args->non_table_or_wall_points->at( i ).at(0) << ", " << args->non_table_or_wall_points->at( i ).at(1) << endl;
			}
			
			args->arm_clusters_2d_points->push_back( cluster_points );
			
			cloud_cluster->width = cloud_cluster->points.size ();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;
			args->arm_clusters_3d_points->push_back( cloud_cluster );
			(*(args->validated_cluster)) = 0;
			if(args->verbose){
				cout << "PointCloud representing the Cluster " << cloud_cluster->points.size () << " data points." << endl;
			}
			// end dbscan version
		}else if(non_table_or_wall_3d_cloud->size() > 0){
			// Do the PCL blob search for the arm
			// http://pointclouds.org/documentation/tutorials/cluster_extraction.php
			std::vector<pcl::PointIndices> cluster_indices;
		
			pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
			tree->setInputCloud (non_table_or_wall_3d_cloud);
	
			pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
			ec.setClusterTolerance (eps);
			ec.setMinClusterSize (1000);
			ec.setMaxClusterSize (50000);
			ec.setSearchMethod (tree);
			ec.setInputCloud (non_table_or_wall_3d_cloud);
			ec.extract (cluster_indices);

	
			for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end() && (*(args->validated_cluster)) < 0; ++it){
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
				vector< vector<double> > cluster_points;
				
				bool is_valid_cluster = false;
				
				for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
					pcl::PointXYZ sample_point = non_table_or_wall_3d_cloud->points[*pit];
					cloud_cluster->points.push_back(sample_point);
					cluster_points.push_back(args->non_table_or_wall_points->at( *pit ));
			
			
					if((*(args->validated_cluster)) == -1 && found_jaco_tag && sample_jaco_point.at(0) == sample_point.x && sample_jaco_point.at(1) == sample_point.y && sample_jaco_point.at(2) == sample_point.z){
						is_valid_cluster = true;
					}
			
				}
				if(is_valid_cluster){
					(*(args->validated_cluster)) = args->arm_clusters_2d_points->size();
					args->arm_clusters_2d_points->push_back( cluster_points );
		
					cloud_cluster->width = cloud_cluster->points.size ();
					cloud_cluster->height = 1;
					cloud_cluster->is_dense = true;
					args->arm_clusters_3d_points->push_back( cloud_cluster );
		
					if(args->verbose){
						cout << "PointCloud representing the Cluster (" << j << "): " << cloud_cluster->points.size () << " data points." << endl;
					}
					break;
				}
				j++;
			}
		
			// end PCL ransac blob findarm
		
		}
		
		// arm points found using some method, now let's decompose.
		if((*(args->validated_cluster)) > -1){
			
			vector< vector<double> > input;
			vector<double> point;
			pcl::PointXYZ existing;			
			for(i = 0; i < args->arm_clusters_3d_points->at( (*(args->validated_cluster)) )->size(); i++){
				existing = args->arm_clusters_3d_points->at( (*(args->validated_cluster)) )->at(i);
				point.clear();
				point.push_back(existing.x);
				point.push_back(existing.y);
				point.push_back(existing.z);
				
				input.push_back(point);
			}
			int initial_centroid_suggestion = 30;
			
			kmeans_cluster_and_centroid(&input, args->arm_skeleton_centroids, initial_centroid_suggestion, 0, args->arm_skeleton_assignments, args->cluster_error_cutoff, false);
			//cout << "Found " << args->arm_skeleton_centroids->size() << " skeleton centroids for " << input.size() << " inputs" << endl;
			if(args->viz_selection == PCL_ARM_SKELETON && args->arm_skeleton_assignments->size() > 0){
				vector<int> assignment_vals;
				for(i = 0; i < args->arm_skeleton_assignments->size(); i++){
					assignment_vals.push_back(args->arm_skeleton_assignments->at(i));
				}
				vector<vector<short>> colors;
				vector<short> white;
				white.push_back(0xff);
				white.push_back(0xff);
				white.push_back(0xff);
				if(*(args->show_viz_cluster_colors)){
					get_unique_colors(args->arm_skeleton_centroids->size(), &colors);
				}else{
					for(i = 0; i < args->arm_skeleton_centroids->size(); i++){
						colors.push_back(white);
					}
				}
				
				pcl_viz_this_vector_of_points(args->pcl_viz_input_ready, args->arm_skeleton_centroids, args->pcl_viz_cloud_input, &colors);
			}
			if((*(args->save_jaco_skeleton_frames)) > 0){
				FILE *out;
				pcl::PointXYZ point3d;
				(*(args->save_jaco_skeleton_frames))--;
				out = fopen(ARM_SKELETON_POINT_FILE, "a");
				cout << "writing skeleton: " << (*(args->save_jaco_skeleton_frames)) << endl;
				
				for(i = 0; i < args->arm_skeleton_centroids->size(); i++){
					point3d.x = args->arm_skeleton_centroids->at(i).at(0);
					point3d.y = args->arm_skeleton_centroids->at(i).at(1);
					point3d.z = args->arm_skeleton_centroids->at(i).at(2);
					fprintf(out, "%f,%f,%f\n", point3d.x, point3d.y, point3d.z);
				}
				fprintf(out, "======================\n");
				fclose(out);
			}
		}
		
	}
	
	
}


void *perform_frame_combinations(void *thread_args){

	struct temporal_smoothing_args *in_args = (struct temporal_smoothing_args *) thread_args;

	int extra_frames = (*(in_args->additional_color_match_frames_to_combine));
	
	in_args->combined->clear();
	// load this round into combined list
	for(int i = 0; i < in_args->matches->size(); i++){
		in_args->combined->push_back(in_args->matches->at(i));
	}
	// load points from previous rounds into combined list
	for(int i = 0; i < extra_frames && i < in_args->previous_frames->size(); i++){
		// load points from specific frame
		for(int j = 0; j < in_args->previous_frames->at(i).size(); j++){
			in_args->combined->push_back(in_args->previous_frames->at(i).at(j));
		}
	}
	if(extra_frames > 0){
		// update previous-frames to include this one. higher = newer, lower = older.

		// if we're maxed out or over the limit (if it changes), delete the last one.
		while(in_args->previous_frames->size() >= extra_frames){
			in_args->previous_frames->erase(in_args->previous_frames->begin());
		}
		// add new frame's data to last slot
		int new_frame_index = in_args->previous_frames->size();
		
		in_args->previous_frames->push_back(  vector< vector<double> >(0) );
	
		for(int i = 0; i < in_args->matches->size(); i++){
			in_args->previous_frames->at(new_frame_index).push_back(in_args->matches->at(i));
		}
	}
}


class ImageConverter{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	ros::Subscriber pcl_sub_;
	int frames_to_skip;
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

	vector< vector< vector<double> > > jaco_arm_matched_points_2d_previous_rounds;

	vector< vector< vector<double> > > all_3d_points_previous_rounds;
	vector< vector< vector<double> > > map_2d_previous_rounds;
	
	pthread_mutex_t centroid_mutex;
	pthread_mutex_t structures_mutex;

	public:
	ImageConverter() : 
	it_(nh_){
		frames_to_skip = 0;
		// Create a ROS subscriber for the input point cloud, contains XYZ, RGB
		pthread_mutex_init(&structures_mutex, NULL);
		pthread_mutex_lock(&structures_mutex);

		reset();
		pcl_sub_ = nh_.subscribe (DEFAULT_KINECT_TOPIC, 1, &ImageConverter::cloudCb, this);
		
		pthread_mutex_unlock(&structures_mutex);
		cv::namedWindow(OPENCV_WINDOW);
	}

	~ImageConverter(){
		cv::destroyWindow(OPENCV_WINDOW);
	}

	void reset(){
		object_centroids_2d.clear();
		object_centroids_3d.clear();
		
		object_matched_points_2d_previous_rounds.clear();
		object_matched_points_3d_previous_rounds.clear();

		jaco_tag_matched_points_2d_previous_rounds.clear();
		jaco_tag_matched_points_3d_previous_rounds.clear();

		jaco_arm_matched_points_2d_previous_rounds.clear();

		jaco_tag_centroids_2d.clear();
		jaco_tag_centroids_3d.clear();

	}

	void set_args(struct viz_thread_args *viz_args){
		pthread_mutex_lock(&structures_mutex);
		args = viz_args;
		reset();
		pcl_sub_ = nh_.subscribe(args->kinect_topic, 1, &ImageConverter::cloudCb, this);
		pthread_mutex_unlock(&structures_mutex);
	}

	bool find_match_by_color(cv::Mat *im_matrix, pcl::PointCloud<pcl::PointXYZRGB> *cloud, int x, int y, vector< vector<double> > *matched_points_2d, vector< vector<double> > *matched_points_3d, struct rgb_set *color_set, bool verbose){
		pcl::PointXYZRGB *point;
		
		// Cloud is (columns, rows)
		point = &cloud->at(x, y);

		Eigen::Vector3i rgb = point->getRGBVector3i();

		// im_matrix is (rows, columns)
		im_matrix->at<cv::Vec3b>(y,x)[0] = rgb[2];
		im_matrix->at<cv::Vec3b>(y,x)[1] = rgb[1];
		im_matrix->at<cv::Vec3b>(y,x)[2] = rgb[0];
		bool match = false;
		for(int i = 0; !match && i < color_set->num_colors; i++){
			match |= do_pixel_test(x, y, im_matrix, &(color_set->colors[i]), matched_points_2d, matched_points_3d, cloud, args->max_interested_distance, DEFAULT_MIN_INTERESTED_DISTANCE, args->visible_angle);
		}
		
		if(args->draw_depth_filter){
			apply_distance_filter(im_matrix, x, y, cloud);
		}
		return match;
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
	

	void color_pixels(cv::Mat *im_matrix, vector< vector<double> > *points, struct rgb *color){
		int x,y;
		for(int i = 0; i < points->size(); i++){
			x = points->at(i).at(0);
			y = points->at(i).at(1);
			if(x < 0 || y < 0 || x >= im_matrix->cols || y >= im_matrix->rows){
				return;
			}
			im_matrix->at<cv::Vec3b>(y,x)[0] = color->b;
			im_matrix->at<cv::Vec3b>(y,x)[1] = color->g;
			im_matrix->at<cv::Vec3b>(y,x)[2] = color->r;
		}
	}

	void add_point_if_possible(queue< struct point_pairs > *point_queue, unordered_map<int, bool> *map, vector<double> xy, int diff_x, int diff_y, cv::Mat *im_matrix, point_type type, int iter_count, bool draw_attempt){
		if(xy.at(0) + diff_x >= im_matrix->cols || xy.at(0) + diff_x < 0 || xy.at(1) + diff_y >= im_matrix->rows || xy.at(1) + diff_y < 0){
			return;
		}
	
		vector<double> new_xy;
		new_xy.push_back(xy.at(0) + diff_x);
		new_xy.push_back(xy.at(1) + diff_y);
		int key = get_key_from_coordinate(new_xy, im_matrix->rows);
		if(map->find(key) == map->end()){
			if(draw_attempt){
				vector< vector<double> > tried;
				tried.push_back(new_xy);
				color_pixels(im_matrix, &tried, &jaco_arm_tried_color);
			}
			
			struct point_pairs pair;
			pair.parent = xy;
			pair.candidate = new_xy;
			pair.type = type;
			pair.up_iterations = iter_count;
			(*map)[key] = true;

			point_queue->push(pair);
		}
	}

	/*
	Adjustments to get coordinate space mapped correctly to Kinect according to https://msdn.microsoft.com/en-us/library/dn785530.aspx
	
	The origin (x=0, y=0, z=0) is located at the center of the IR sensor on Kinect
	X grows to the sensor’s left
	Y grows up (note that this direction is based on the sensor’s tilt)
	Z grows out in the direction the sensor is facing
	1 unit = 1 meter

	Was receiving X growing to right, and y growing down
	*/
	void apply_kinect_space_flip(struct xyz *xyz){
		xyz->x *= -1;
		xyz->y *= -1;
	}



	//void cloudCb (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& input){
	void cloudCb (const sensor_msgs::PointCloud2ConstPtr& input){
		if(args->terminate){
			cv::destroyWindow(OPENCV_WINDOW);
			return;
		}
		
		bool verbose = args->verbose;
		
		// skip every-other frame for faster rendering
		if(frames_to_skip > 0){
			frames_to_skip--;
			cout << "skipping " << frames_to_skip << endl;
			return;
		}
		frames_to_skip = args->skip_frames;
		
		pthread_mutex_lock(&structures_mutex);
		
		// ROS message to pcl::PointCloud<pcl::PointXYZRGB> *cloud;
  		pcl::fromROSMsg (*input, *(args->cloud));
		
		int i, j, x, y;

		cv::Mat im_matrix(args->cloud->height, args->cloud->width, CV_8UC3);
		
		vector< vector<double> > all_3d_points;
		vector< vector<double> > map_back_to_2d;	
		
		// store 2d matches and 3d matches
		vector< vector<double> > object_matched_points_2d;
		vector< vector<double> > object_matched_points_3d;


		vector< vector<double> > jaco_tag_matched_points_2d;
		vector< vector<double> > jaco_tag_matched_points_3d;
		

		double xyz[3];
		double dist;
		bool match;
		vector<double> temp_xyz;
		temp_xyz.resize(3, 0);
		vector<double> temp_xy;
		temp_xy.resize(2, 0);
		bool is_valid_point;
		for (y = 0; y < im_matrix.rows; y++) {
			for (x = 0; x < im_matrix.cols; x++) {
				
				get_xyz_from_xyzrgb(x, y, args->cloud, xyz);
				is_valid_point = is_valid_xyz(xyz); 

				dist = is_valid_point ? vector_length_3d(xyz) : -1;
				// Grab all 3d points in cloud that are close enough (avoid back wall)
				if(is_valid_point && dist < args->max_interested_distance){
					temp_xyz.at(0) = xyz[0];
					temp_xyz.at(1) = xyz[1];
					temp_xyz.at(2) = xyz[2];
					all_3d_points.push_back(temp_xyz);
					// Now indices are matched between the 3d and 2d points.
					temp_xy.at(0) = x;
					temp_xy.at(1) = y;
					map_back_to_2d.push_back(temp_xy);
				}
				
				// find orange_bottle_cylinder
				match = find_match_by_color(&im_matrix, args->cloud, x, y, &object_matched_points_2d, &object_matched_points_3d, &args->orange_bottle_colors, verbose);

				// find jaco tag
				match |= find_match_by_color(&im_matrix, args->cloud, x, y, &jaco_tag_matched_points_2d, &jaco_tag_matched_points_3d, &blue_tag, verbose);
				

				if(args->highlight_visible_area && is_valid_point){
					
					if(!is_visible_angle(xyz[0], xyz[1], xyz[2], args->visible_angle)){
						// add a green tinge
						im_matrix.at<cv::Vec3b>(y,x)[1] = MIN(im_matrix.at<cv::Vec3b>(y,x)[1] + 100, 255);
					}else if(dist > args->max_interested_distance){
						im_matrix.at<cv::Vec3b>(y,x)[0] = MIN(im_matrix.at<cv::Vec3b>(y,x)[0] + 100, 255);
					}
				}
			}
		}
		

		// merge frames, temporal smoothing
		vector< vector<double> > all_3d_points_combined;
		struct temporal_smoothing_args merge_1;
		merge_1.combined = &all_3d_points_combined;
		merge_1.matches = &all_3d_points;
		merge_1.previous_frames = &all_3d_points_previous_rounds;
		merge_1.additional_color_match_frames_to_combine = &(args->additional_color_match_frames_to_combine);
		pthread_t merge_1_thread;
		pthread_create(&merge_1_thread, NULL, perform_frame_combinations, (void *) &merge_1);
		

		vector< vector<double> > map_2d_combined;
		struct temporal_smoothing_args merge_2;
		merge_2.combined = &map_2d_combined;
		merge_2.matches = &map_back_to_2d;
		merge_2.previous_frames = &map_2d_previous_rounds;
		merge_2.additional_color_match_frames_to_combine = &(args->additional_color_match_frames_to_combine);
		pthread_t merge_2_thread;
		pthread_create(&merge_2_thread, NULL, perform_frame_combinations, (void *) &merge_2);
		

		vector< vector<double> > object_matched_points_2d_combined;
		struct temporal_smoothing_args merge_3;
		merge_3.combined = &object_matched_points_2d_combined;
		merge_3.matches = &object_matched_points_2d;
		merge_3.previous_frames = &object_matched_points_2d_previous_rounds;
		merge_3.additional_color_match_frames_to_combine = &(args->additional_color_match_frames_to_combine);
		pthread_t merge_3_thread;
		pthread_create(&merge_3_thread, NULL, perform_frame_combinations, (void *) &merge_3);


		vector< vector<double> > object_matched_points_3d_combined;
		struct temporal_smoothing_args merge_4;
		merge_4.combined = &object_matched_points_3d_combined;
		merge_4.matches = &object_matched_points_3d;
		merge_4.previous_frames = &object_matched_points_3d_previous_rounds;
		merge_4.additional_color_match_frames_to_combine = &(args->additional_color_match_frames_to_combine);
		pthread_t merge_4_thread;
		pthread_create(&merge_4_thread, NULL, perform_frame_combinations, (void *) &merge_4);

		


		vector< vector<double> > jaco_tag_matched_points_2d_combined;
		struct temporal_smoothing_args merge_5;
		merge_5.combined = &jaco_tag_matched_points_2d_combined;
		merge_5.matches = &jaco_tag_matched_points_2d;
		merge_5.previous_frames = &jaco_tag_matched_points_2d_previous_rounds;
		merge_5.additional_color_match_frames_to_combine = &(args->additional_color_match_frames_to_combine);
		pthread_t merge_5_thread;
		pthread_create(&merge_5_thread, NULL, perform_frame_combinations, (void *) &merge_5);

		
		vector< vector<double> > jaco_tag_matched_points_3d_combined;
		struct temporal_smoothing_args merge_6;
		merge_6.combined = &jaco_tag_matched_points_3d_combined;
		merge_6.matches = &jaco_tag_matched_points_3d;
		merge_6.previous_frames = &jaco_tag_matched_points_3d_previous_rounds;
		merge_6.additional_color_match_frames_to_combine = &(args->additional_color_match_frames_to_combine);
		pthread_t merge_6_thread;
		pthread_create(&merge_6_thread, NULL, perform_frame_combinations, (void *) &merge_6);

		
		if(verbose){
			cout << "waiting on temporal smoothing threads" << endl;
		}
		pthread_join(merge_1_thread, NULL);
		pthread_join(merge_2_thread, NULL);
		pthread_join(merge_3_thread, NULL);
		pthread_join(merge_4_thread, NULL);
		pthread_join(merge_5_thread, NULL);
		pthread_join(merge_6_thread, NULL);
		if(verbose){
			cout << "Done" << endl;
		}


		vector< vector< vector<double> > > arm_clusters_2d_points;
		vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > arm_clusters_3d_points;
		int validated_cluster = -1;

		vector< vector<double> > table_points;
		vector< vector<double> > wall_points;
		vector< vector<double> > non_table_or_wall_points;

		arma::Row<size_t> arm_skeleton_assignments;
		vector< vector<double> > arm_skeleton_centroids;
		
		// do find-arm
		struct find_arm_args find_arm_args;
		find_arm_args.verbose = verbose;
		find_arm_args.map_2d_combined = &map_2d_combined;
		find_arm_args.jaco_tag_matched_points_3d_combined = &jaco_tag_matched_points_3d_combined;
		find_arm_args.all_3d_points_combined = &all_3d_points_combined;
		find_arm_args.arm_clusters_2d_points = &arm_clusters_2d_points;
		find_arm_args.table_points = &table_points;
		find_arm_args.wall_points = &wall_points;
		find_arm_args.non_table_or_wall_points = &non_table_or_wall_points;

		find_arm_args.arm_clusters_3d_points = &arm_clusters_3d_points;
		find_arm_args.validated_cluster = &validated_cluster;
		find_arm_args.use_dbscan = args->use_dbscan;
		
		find_arm_args.viz_selection = args->viz_selection;
		find_arm_args.pcl_viz_input_ready = args->pcl_viz_input_ready;
		find_arm_args.pcl_viz_cloud_input = args->pcl_viz_cloud_input;
		
		find_arm_args.arm_skeleton_centroids = &arm_skeleton_centroids;
		find_arm_args.arm_skeleton_assignments = &arm_skeleton_assignments;
		find_arm_args.cluster_error_cutoff = args->cluster_error_cutoff;
		find_arm_args.save_jaco_skeleton_frames = &(args->save_jaco_skeleton_frames);
		find_arm_args.show_viz_cluster_colors = &(args->show_viz_cluster_colors);
		

		pthread_t do_find_arm_thread;
		pthread_create(&do_find_arm_thread, NULL, do_find_arm, (void *) &find_arm_args);
	
		
		if(verbose){
			cout << "post: " << object_matched_points_2d.size() << endl;
		}
		

		if(args->detection_algorithm == KMEANS){
			double cutoff = 0.1;
			arma::Row<size_t> assignments;
			// Arbitrary 5 initially, maybe we know we're looking for n of whatever.
			int max_centroids_to_try = 5;
			kmeans_cluster_and_centroid(&object_matched_points_3d_combined, &object_centroids_3d, max_centroids_to_try, args->num_objects_in_scene, &assignments, &cutoff, verbose);
		
			kmeans_cluster_and_centroid(&jaco_tag_matched_points_3d_combined, &jaco_tag_centroids_3d, max_centroids_to_try, args->num_jaco_arms_in_scene, &assignments, &cutoff, verbose);
		}else if(args->detection_algorithm == HISTOGRAM){
		
			compute_centroids(&object_matched_points_2d, &object_matched_points_3d, &object_centroids_2d, &object_centroids_3d, verbose);
			compute_centroids(&jaco_tag_matched_points_2d, &jaco_tag_matched_points_3d, &jaco_tag_centroids_2d, &jaco_tag_centroids_3d, verbose);
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
				
				apply_kinect_space_flip(&(args->object_xyz[i]));
				
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

				apply_kinect_space_flip(&(args->jaco_tag_xyz[i]));
				
				args->jaco_distances[i] = vector_length_3d_struct(& args->jaco_tag_xyz[i]);
				
				if(verbose){
					cout << "Distance to JACO tag (" << i << ") : " << args->jaco_distances[i] << endl;
				}



				
			}
		}

		if(verbose){
			cout << "waiting on do find arm thread" << endl;
		}
		pthread_join(do_find_arm_thread,NULL); 

		if(args->viz_selection == PCL_JUST_ARM && arm_skeleton_assignments.size() > 0 && validated_cluster >= 0){
			vector<vector<short>> colors;
			vector<short> white;
			white.push_back(0xff);
			white.push_back(0xff);
			white.push_back(0xff);
			for(i = 0; i < arm_skeleton_assignments.size(); i++){
				vector<short> color;
				if(args->show_viz_cluster_colors){
					color = get_hex_color(arm_skeleton_assignments.at(i));
				}else{
					color = white;
				}
				colors.push_back(color);
			}
			
			//load_white(arm_clusters_3d_points.at(validated_cluster)->size(), &colors);
			pcl_viz_this_cloud(args->pcl_viz_input_ready, arm_clusters_3d_points.at(validated_cluster), args->pcl_viz_cloud_input, &colors);
		}
		
		if(args->highlight_table){
			color_pixels(&im_matrix, &table_points, &table_color);
			color_pixels(&im_matrix, &wall_points, &wall_color);
			color_pixels(&im_matrix, &non_table_or_wall_points, &misc_color);
		}
		if(args->find_arm){
			/*
			// Commenting out blue coloring in favor of cluster-based
			if(validated_cluster >= 0){
				color_pixels(&im_matrix, &(arm_clusters_2d_points.at(validated_cluster)), &jaco_arm_match_color);
			}else{
				for(i = 0; i < arm_clusters_2d_points.size(); i++){
					color_pixels(&im_matrix, &(arm_clusters_2d_points.at(i)), &jaco_arm_match_color);
				}
			}
			*/
		
			if(validated_cluster >= 0){
				// arm_clusters_2d_points
				// arma::Row<size_t> arm_skeleton_assignments;
				// vector< vector<double> > arm_skeleton_centroids;
				int assignment = 0, x = 0, y = 0;
				//cout << "has " << arm_skeleton_assignments.n_elem << " elements" << endl;
				struct rgb color;
				FILE *out;
				bool writing_out = false;
				if(args->save_jaco_pcl_frames > 0){
					writing_out = true;
					args->save_jaco_pcl_frames--;
					out = fopen(ARM_PCL_POINT_FILE, "a");
					cout << "writing pcl: " << args->save_jaco_pcl_frames << endl;
			
				}
				pcl::PointXYZ point3d;
				for(i = 0; i < arm_skeleton_assignments.n_elem; i++){
					assignment = arm_skeleton_assignments.at(i);
					vector<short> color = get_hex_color(assignment);
					
					x = arm_clusters_2d_points.at(validated_cluster).at(i).at(0);
					y = arm_clusters_2d_points.at(validated_cluster).at(i).at(1);
					im_matrix.at<cv::Vec3b>(y,x)[0] = color.at(2);
					im_matrix.at<cv::Vec3b>(y,x)[1] = color.at(1);
					im_matrix.at<cv::Vec3b>(y,x)[2] = color.at(0);
	
					if(writing_out){		
						point3d = arm_clusters_3d_points.at(validated_cluster)->at(i);
						fprintf(out, "%f,%f,%f\n", point3d.x, point3d.y, point3d.z);
					}
										
				}
				if(writing_out){
					fprintf(out, "======================\n");
					fclose(out);
				}
			}
		}
		
		if(args->draw_pixel_match_color){
			color_pixels(&im_matrix, &object_matched_points_2d_combined, &match_color);
			color_pixels(&im_matrix, &jaco_tag_matched_points_2d_combined, &jaco_match_color);
		}
		
		//cout << "Num centroids: " <<  object_centroids_3d.size() << ", " << im_matrix.rows << " by " << im_matrix.cols << endl;
		double x_2d,y_2d;
		for(i = 0; i < object_centroids_3d.size(); i++){
			// http://stackoverflow.com/questions/6139451/how-can-i-convert-3d-space-coordinates-to-2d-space-coordinates
			get_2d_coord_for_3d_depth_coord(&x_2d, &y_2d, args->cloud, &object_centroids_3d.at(i));

			// Draw an circle on the video stream around the 2d centroids
			cv::circle(im_matrix, cv::Point(x_2d, y_2d), 20, CV_RGB(match_color.r,match_color.g,match_color.b));
		}
		
		for(i = 0; i < jaco_tag_centroids_3d.size(); i++){
			get_2d_coord_for_3d_depth_coord(&x_2d, &y_2d, args->cloud, &jaco_tag_centroids_3d.at(i));
			
			// Draw an circle on the video stream around the 2d centroids
			cv::circle(im_matrix, cv::Point(x_2d, y_2d), 20, CV_RGB(jaco_match_color.r,jaco_match_color.g,jaco_match_color.b));
			
		}
		
		// Update GUI Window
		
		cv::imshow(OPENCV_WINDOW, im_matrix);
		cv::waitKey(3);
		pthread_mutex_unlock(&structures_mutex);
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
		int minimum_points_per_cluster = 25;
		
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
	
	ImageConverter ic;
	ic.set_args(viz_args);
	ros::Rate r(10);

	char *current_topic = viz_args->kinect_topic;
	
	while(!viz_args->terminate){
		if(viz_args->kinect_topic != current_topic){
			ic.set_args(viz_args);
			current_topic = viz_args->kinect_topic;
		}
		ros::spinOnce();
		r.sleep();
	}
	cout << endl << "Terminating viz" << endl;
	cv::destroyWindow(OPENCV_WINDOW);
}




#endif

