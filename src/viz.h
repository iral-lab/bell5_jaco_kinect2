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

struct viz_thread_args{
	int *argc;
	char ***argv;
	bool terminate;
};

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


double euclid_distance_3d(vector<double> a, vector<double> b){
	// compute integer distance between two points in 3d
	return (double) sqrtf((a.at(0) - b.at(0)) * (a.at(0) - b.at(0)) + (a.at(1) - b.at(1)) * (a.at(1) - b.at(1)) + (a.at(2) - b.at(2)) * (a.at(2) - b.at(2)) );
}

int euclid_distance_3d_not_vec(double *a, double *b){
	// compute integer distance between two points in 3d as a[] and b[]
	return (int) sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2]) );
}

double vector_length_3d(double *xyz){
	return (double) sqrtf(xyz[0]*xyz[0] + xyz[1]*xyz[1] + xyz[2]*xyz[2]);
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

void apply_distance_filter(cv::Mat *im_matrix, int h, int w, pcl::PointCloud<pcl::PointXYZRGB> *cloud){
	double dist;
	double xyz[3];
	get_xyz_from_xyzrgb(h, w, cloud, xyz);
	
	dist = vector_length_3d(xyz);
	short color = color_normalize(dist, 3.0, 255);
	color = MIN(255, color);
	color = MAX(0, color);
	im_matrix->at<cv::Vec3b>(h,w)[0] = color;
	im_matrix->at<cv::Vec3b>(h,w)[1] = color;
	im_matrix->at<cv::Vec3b>(h,w)[2] = color;
}

bool is_valid_xyz(double *xyz){
	return !std::isnan(xyz[0]) && !std::isnan(xyz[1]) && !std::isnan(xyz[2]);
}

bool do_pixel_test(int x, int y, cv::Mat *image, struct rgb *desired, vector< vector<int> > *matches_2d, vector< vector<double> > *matches_3d, pcl::PointCloud<pcl::PointXYZRGB> *cloud){
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
		
		matches_2d->push_back( vector<int>(2) );
		matches_2d->at(matches_size).at(0) = x;
		matches_2d->at(matches_size).at(1) = y;
		
		//cout << "3d xyz: " << xyz[0] << "," << xyz[1] << "," << xyz[2] << endl;
		//cout << "3d match: " << matches_3d->at(matches_size).at(0) << "," << matches_3d->at(matches_size).at(1) << "," << matches_3d->at(matches_size).at(2) << endl;
		//cout << "2d match: " << matches_2d->at(matches_size).at(0) << "," << matches_2d->at(matches_size).at(1) << endl;
		match = true;
	}
	return match;
}

class ImageConverter{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	ros::Subscriber pcl_sub_;
	int frames;
	struct viz_thread_args *args;

	// store the centroids in both xyz and 2d image
	vector< vector<int> > centroids_2d;
	vector< vector<double> > centroids_3d;
	
	pthread_mutex_t centroid_mutex;

	public:
	ImageConverter() : 
	it_(nh_){
		frames = 0;
		// Create a ROS subscriber for the input point cloud, contains XYZ, RGB
		pcl_sub_ = nh_.subscribe ("/kinect2/qhd/points", 1, &ImageConverter::cloudCb, this);
	
		centroids_2d.clear();
		centroids_3d.clear();
		
		cv::namedWindow(OPENCV_WINDOW);
	}

	~ImageConverter(){
		cv::destroyWindow(OPENCV_WINDOW);
	}

	void set_args(struct viz_thread_args *viz_args){
		args = viz_args;
	}
	
	//void cloudCb (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& input){
	void cloudCb (const sensor_msgs::PointCloud2ConstPtr& input){
		if(args->terminate){
			cv::destroyWindow(OPENCV_WINDOW);
			return;
		}
		
		bool verbose = false;
		
		// skip every-other frame for faster rendering
		if(++frames % 2 == 0){
			//cout << "skipping " << frames << endl;
			return;
		}
		
		
		pcl::PointCloud<pcl::PointXYZRGB> cloud;
  		pcl::fromROSMsg (*input, cloud);
		pcl::PointXYZRGB *point;
		
		int i, j, x, y;

		cv::Mat im_matrix(cloud.height, cloud.width, CV_8UC3);
		
		// store 2d matches and 3d matches
		vector< vector<int> > matched_points_2d;
		vector< vector<double> > matched_points_3d;
		bool match = false;
		int new_h, new_w;
		if(verbose)
			cout << "about to find colors" << endl;
		
		for (int y = 0; y < im_matrix.rows; y++) {
			for (int x = 0; x < im_matrix.cols; x++) {
				// Cloud is (columns, rows)
				point = &cloud.at(x, y);

				Eigen::Vector3i rgb = point->getRGBVector3i();

				// im_matrix is (rows, columns)
				im_matrix.at<cv::Vec3b>(y,x)[0] = rgb[2];
				im_matrix.at<cv::Vec3b>(y,x)[1] = rgb[1];
				im_matrix.at<cv::Vec3b>(y,x)[2] = rgb[0];
				
				match = do_pixel_test(x, y, &im_matrix, &orange, &matched_points_2d, &matched_points_3d, &cloud);
				if(match){
					// do pixel shading
					im_matrix.at<cv::Vec3b>(y,x)[0] = 255;
					im_matrix.at<cv::Vec3b>(y,x)[1] = 255;
					im_matrix.at<cv::Vec3b>(y,x)[2] = 255;
				}
				
				//apply_distance_filter(&im_matrix, x, y, &cloud);
			}
		}
		
		
		if(verbose)
			cout << "post: " << matched_points_2d.size() << endl;
		/*		
		if(frames > 1){
			return;
		}
		*/

		//pthread_mutex_lock(&centroid_mutex);
		compute_centroids(&matched_points_2d, &matched_points_3d, &centroids_2d, &centroids_3d, verbose);
		//pthread_mutex_unlock(&centroid_mutex);
		
		for(i = 0; i < centroids_2d.size(); i++){
			// Draw an circle on the video stream around the 2d centroids
			cv::circle(im_matrix, cv::Point(centroids_2d.at(i).at(0), centroids_2d.at(i).at(1)), 20, CV_RGB(255,0,0));
		}
		
		if(centroids_3d.size() == 2){
			double c0_xyz[3];
			double c1_xyz[3];
			
			c0_xyz[0] = centroids_3d.at(0).at(0);
			c0_xyz[1] = centroids_3d.at(0).at(1);
			c0_xyz[2] = centroids_3d.at(0).at(2);
			if(verbose)
				cout << "\tdistance between centroids: " << euclid_distance_3d_not_vec(c0_xyz, c1_xyz) << endl;
		}
		
		// Update GUI Window
		
		cv::imshow(OPENCV_WINDOW, im_matrix);
		cv::waitKey(3);
	}

	void compute_centroids(vector< vector<int> > *matches_2d, vector< vector<double> > *matches_3d, vector< vector<int> > *centroids_2d, vector< vector<double> > *centroids_3d, bool verbose){
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
		
		vector< vector< vector<int> > > clusters_2d;
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
				clusters_2d.push_back( vector< vector<int> >(0) );
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
		int x_sum_2d, y_sum_2d;
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
			centroids_2d->push_back(vector<int>(2));
			centroids_2d->at(num_centroids).at(0) = (int) x_sum_2d;
			centroids_2d->at(num_centroids).at(1) = (int) y_sum_2d;

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





