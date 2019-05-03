#include <ros/ros.h>
#include <iostream>
#include <cmath>

#include "sensor_msgs/image_encodings.h"
#include <image_transport/image_transport.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();
  cv::Mat img=cv::imread("\\home\\catkin_ws\\src\\torrilds_package\\src\\templates\\test.jpg",0);
  cv::imshow("tis",img);
  ros::spin();
  cv::destroyWindow("view");
}
