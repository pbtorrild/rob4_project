#include <ros/ros.h>
#include <iostream>

#include "sensor_msgs/image_encodings.h"
#include <image_transport/image_transport.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
int file_num=0;
void imageCallback(const sensor_msgs::ImageConstPtr msg){
	cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
  //image pros
  cv::Mat input = cv_ptr->image;

	std::string file_path="/home/ros/catkin_ws/src/torrilds_package/src/templates";
	std::string file_name="train_data_";
	std::string file_number=std::to_string(file_num);
	std::string file_type=".jpg";
	std::string full_path=file_path+file_name+file_number+file_type;

	std::cout << full_path << '\n';

	file_num++;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  //image_transport::Subscriber sub = it.subscribe("camera/color/image_raw", 1, imageCallback);
	cv::Mat test=cv::imread("/home/ros/catkin_ws/src/torrilds_package/src/templates/test.jpg", CV_LOAD_IMAGE_COLOR);
	cv::imshow("view",test);
	cv::waitKey();
	ros::spin();
  cv::destroyWindow("view");
}
