#include <vector>
#include <pthread.h>
#include <cmath>

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

struct rgb{
	short r;
	short g;
	short b;
};


// orange color: #ef5e25
struct rgb orange = {239, 94, 37};

// bottle color: #45c5d2
struct rgb bottle = {69, 197, 210};


bool colors_are_similar(struct rgb *x, struct rgb *y){
	double distance = sqrt( (x->r - y->r)*(x->r - y->r) + (x->g - y->g)*(x->g - y->g) + (x->b - y->b)*(x->b - y->b) );
	// epsilon subject to change here, was trial and error early on.
	return distance < 50;
}


static const std::string OPENCV_WINDOW = "Image window";
#define BATCH_SIZE 1000;

void do_pixel_test(int i, int j, cv::Mat *image, struct rgb *desired, vector< vector<int> > *matches){
	struct rgb test;
	test.r = (*image).at<cv::Vec3b>(i,j)[2];
	test.g = (*image).at<cv::Vec3b>(i,j)[1];
	test.b = (*image).at<cv::Vec3b>(i,j)[0];
	
	int matches_size = matches->size();
	if(colors_are_similar(desired, &test)){
		matches->push_back( vector<int>(0) );
		matches->at(matches_size).push_back(i);
		matches->at(matches_size).push_back(j);
	}	
}

int euclid_distance_2d(vector<int> a, vector<int> b){
	// compute integer distance between two points
	return (int) sqrt((a.at(0) - b.at(0)) * (a.at(0) - b.at(0)) + (a.at(1) - b.at(1)) * (a.at(1) - b.at(1)));
}

double vector_length_3d(double x, double y, double z){
	return (double) sqrtf(x*x + y*y + z*z);
}



class ImageConverter{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	ros::Subscriber pcl_sub_;

	vector< vector<int> > centroids;
	pthread_mutex_t centroid_mutex;

	public:
	ImageConverter() : 
	it_(nh_){
		// Subscrive to input video feed and publish output video feed
		//image_sub_ = it_.subscribe("/kinect2/hd/image_color_rect", 1, &ImageConverter::imageCb, this);
		

		// Create a ROS subscriber for the input point cloud
		pcl_sub_ = nh_.subscribe ("/kinect2/hd/points", 1, &ImageConverter::cloudCb, this);
	
		cv::namedWindow(OPENCV_WINDOW);
	}

	~ImageConverter(){
		cv::destroyWindow(OPENCV_WINDOW);
	}

	
	//void cloudCb (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& input){
	void cloudCb (const sensor_msgs::PointCloud2ConstPtr& input){
		// skip every-other frame for speed
		if(rand() % 2 == 1){
			return;
		}
		
		pcl::PointCloud<pcl::PointXYZRGB> cloud;
  		pcl::fromROSMsg (*input, cloud);
		pcl::PointXYZRGB *point;
		
		int i, x, y;

		cv::Mat im_matrix(cloud.height, cloud.width, CV_8UC3);
		for (int h=0; h<im_matrix.rows; h++) {
			for (int w=0; w<im_matrix.cols; w++) {
				point = &cloud.at(w, h);

				Eigen::Vector3i rgb = point->getRGBVector3i();

				im_matrix.at<cv::Vec3b>(h,w)[0] = rgb[2];
				im_matrix.at<cv::Vec3b>(h,w)[1] = rgb[1];
				im_matrix.at<cv::Vec3b>(h,w)[2] = rgb[0];
			}
		}
		
		cv::imshow(OPENCV_WINDOW, im_matrix);
		cv::waitKey(3);
		
		//cout << "got packet, H: " << cloud.width << ", W: " << cloud.height << endl;
		//pthread_mutex_lock(&centroid_mutex);
		//for(x = 0; x < cloud.width; x++){
		for(i = 0; i < centroids.size(); i++){
			x = centroids.at(i).at(0);
			y = centroids.at(i).at(1);
			point = &cloud.at(x,y);
			//cout << *point << endl;
			
			// Test for Nan, if item is too close
			if(isnan(point->x)){
				continue;		
			}
			
			cout << "(" << x << "," << y << ") = ( " << point->x << "," << point->y << "," << point->z << "), dist from center (?): " << vector_length_3d((double) point->x, (double) point->y, (double) point->z) << "." << endl;
		}
		//pthread_mutex_unlock(&centroid_mutex);
	}

	void compute_centroids(vector< vector<int> > *matches, vector< vector<int> > *centroids, bool verbose){
		// get all distances between points
		vector<int> distances;
		int i, j, k;
		int num_matches = matches->size();
		for(i = 0; i < num_matches; i++){
			for(j = i+1; j < num_matches; j++){
				distances.push_back(euclid_distance_2d(matches->at(i), matches->at(j)));
			}
		}
	
		// do sort/unique on distance measures
		sort (distances.begin(), distances.end());
		std::vector<int>::iterator it;
		it = std::unique (distances.begin(), distances.end());
		distances.resize( std::distance(distances.begin(),it) );
	
	
		// now distances are unique
		if(verbose)
		cout << "\tFinal distances: " << distances.size() << endl;
	
		// find clusters based on histogram
		int missing = 0, index = 0, current;
		int missing_needed = 200;
		int last = -1;
		int delta, threshold;
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

		int dist;
		bool found_cluster, matched_all;
		vector< vector< vector<int> > > clusters;
		for(i = 0; i < num_matches; i++){
			// find which cluster the point belongs in, or create new cluster
			found_cluster = false;
		
			for(j = 0; !found_cluster && j < clusters.size(); j++){
				matched_all = true;
				for(k = 0; matched_all && k < clusters.at(j).size(); k++){
					dist = euclid_distance_2d(matches->at(i), clusters.at(j).at(k));
					if(dist > threshold){
						matched_all = false;
					}
				}
			
				if(matched_all && clusters.size() > 0){
					clusters.at(j).push_back(matches->at(i));
					found_cluster = true;
				}
			}
			if(!found_cluster){
				// create new cluster
				clusters.push_back( vector< vector<int> >(0) );
				// add this point to the cluster we just made
				clusters.at(clusters.size() - 1).push_back( matches->at(i));
			}
		}
	
		if(verbose)
		cout << "\tNum clusters: " << clusters.size() << endl;
		int minimum_points_per_cluster = 100;
	
		//pthread_mutex_lock(&centroid_mutex);
		centroids->clear();
	
		int x_sum, y_sum, num_centroids, total;
		for(i = 0; i < clusters.size(); i++){
			//cout << "Cluster " << i << " has " << clusters.at(i).size() << endl;
			total = clusters.at(i).size();
			if(total < minimum_points_per_cluster){
				continue;
			}
		
			x_sum = y_sum = 0;
			for(j = 0; j < total; j++){
				x_sum += clusters.at(i).at(j).at(0);
				y_sum += clusters.at(i).at(j).at(1);
				//cout << j << " added " << clusters.at(i).at(j).at(0) << "," << clusters.at(i).at(j).at(1) << " => (" << x_sum << "," << y_sum << ")" << endl;
			}
			num_centroids = centroids->size();
			centroids->push_back(vector<int>(2));
			// NOT SURE WHY X,Y ARE FLIPPED HERE.
			centroids->at(num_centroids).at(1) = (int)(x_sum / total);
			centroids->at(num_centroids).at(0) = (int)(y_sum / total);
			//cout << "centroid computed at: " << centroids->at(num_centroids).at(0) << "," << centroids->at(num_centroids).at(1) << endl;
		}
		//pthread_mutex_unlock(&centroid_mutex);
		if(verbose)
		cout << "\tNum centroids: " << centroids->size() << endl;	
	}
	
	void imageCb(const sensor_msgs::ImageConstPtr& msg){
		// skip every-other frame for speed
		if(rand() % 2 == 1){
			return;
		}
		cv_bridge::CvImagePtr cv_ptr;
		
		
		try{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}catch (cv_bridge::Exception& e){
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		cv::Mat *im_matrix = &(cv_ptr->image);
		
		
		vector< vector<int> > matched_points;

		int i, j;
		for(i = 0; i < im_matrix->rows; i+=2){
			for(j = 0; j < im_matrix->cols; j+=2){
				do_pixel_test(i, j, im_matrix, &orange, &matched_points);
			}
		}

		bool verbose = false;



		if(verbose)
		cout << "post: " << matched_points.size() << endl;

		for(i = 0; i < matched_points.size(); i++){
			(*im_matrix).at<cv::Vec3b>(matched_points.at(i).at(0),matched_points.at(i).at(1))[0] = 255;
			(*im_matrix).at<cv::Vec3b>(matched_points.at(i).at(0),matched_points.at(i).at(1))[1] = 255;
			(*im_matrix).at<cv::Vec3b>(matched_points.at(i).at(0),matched_points.at(i).at(1))[2] = 255;
		}
		
		compute_centroids(&matched_points, &centroids, verbose);

		for(i = 0; i < centroids.size(); i++){
			// Draw an circle on the video stream around the centroids
			cv::circle(*im_matrix, cv::Point(centroids.at(i).at(0), centroids.at(i).at(1)), 20, CV_RGB(255,0,0));
		}

		// Update GUI Window
		cv::imshow(OPENCV_WINDOW, *im_matrix);
		cv::waitKey(3);
	}
};


void handle_viz(int argc, char **argv){
	
	ros::init(argc, argv, "image_converter");
	ImageConverter ic;
	ros::spin();
	
}





