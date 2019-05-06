#include <ros/ros.h>
#include <iostream>
#include <math.h>

#include "sensor_msgs/image_encodings.h"
#include <image_transport/image_transport.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>



void imageCallback(const sensor_msgs::ImageConstPtr msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
  //image pros
  cv::Mat frame = cv_ptr->image;
  //
  // Draw a circle
  cv::circle( frame, cv::Point( frame.rows/2, frame.cols/2 ), 32.0, cv::Scalar( 0, 0, 255 ), 10, 8 );
  cv::imshow("Frame",frame);
  cv::waitKey();
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("camera/color/image_raw", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("view");
}
