
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

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter{
ros::NodeHandle nh_;
image_transport::ImageTransport it_;
image_transport::Subscriber image_sub_;

public:
ImageConverter() : 
	it_(nh_){
		// Subscrive to input video feed and publish output video feed
		image_sub_ = it_.subscribe("/kinect2/hd/image_color_rect", 1, &ImageConverter::imageCb, this);

		cv::namedWindow(OPENCV_WINDOW);
	}

	~ImageConverter(){
		cv::destroyWindow(OPENCV_WINDOW);
	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg){
		cv_bridge::CvImagePtr cv_ptr;
		
		
		try{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}catch (cv_bridge::Exception& e){
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		cv::Mat *im_matrix = &(cv_ptr->image);
		
		// Draw an example circle on the video stream
		if (im_matrix->rows > 60 && im_matrix->cols > 60)
			cv::circle(*im_matrix, cv::Point(50, 50), 10, CV_RGB(255,0,0));
		
		
	

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

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	cout << "got packet, H: " << input->width << ", W: " << input->height << endl;
}

void handle_depth(int argc, char **argv){
	// Initialize ROS
	ros::init (argc, argv, "my_pcl_tutorial");
	ros::NodeHandle nh;

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe ("/kinect2/hd/points", 1, cloud_cb);


	// Spin
	ros::spin ();
	
}



