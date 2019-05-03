#include <ros/ros.h>
#include <iostream>
#include <cmath>

#include "sensor_msgs/image_encodings.h"
#include <image_transport/image_transport.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

void lines(cv::Mat im) {
  cv::Mat input=im;
  cv::Mat frame=im;
  cv::Mat mask;
  int avg_Xs, avg_Ys, avg_Xe, avg_Ye, delta_D;
  double slope, constant, D_desired = 0, D, sum_D, avg_D;
	cv::Mat threshold1;
	cv::cvtColor(im, threshold1, cv::COLOR_BGR2GRAY);
	cv::cvtColor(im, mask, cv::COLOR_BGR2HSV);
	cv::inRange(mask, cv::Scalar(0, 0, 0), cv::Scalar(180, 50, 50), mask);

	cv::Mat element1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(19, 19));
	cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, element1);

	//Canny(mask, mask, 75, 150);
	std::vector<cv::Vec4i> lines;
	cv::HoughLinesP(mask, lines, 1, CV_PI / 180, 50, (input.rows / 3) - 30, 5);

	int sum_Xs = 0, sum_Ys = 0, sum_Xe = 0, sum_Ye = 0;
	for (size_t i = 0; i < lines.size(); i++) {

		cv::Vec4i l = lines[i];
		if (l[1] < frame.rows / 2) {
			sum_Xs = sum_Xs + l[0];
			sum_Ys = sum_Ys + l[1];
			sum_Xe = sum_Xe + l[2];
			sum_Ye = sum_Ye + l[3];
		}
		else {
			sum_Xs = sum_Xs + l[2];
			sum_Ys = sum_Ys + l[3];
			sum_Xe = sum_Xe + l[0];
			sum_Ye = sum_Ye + l[1];
		}
		cv::circle(input, cv::Point(l[0], l[1]), 3, cv::Scalar(0, 0, 255), -1);
		cv::circle(input, cv::Point(l[2], l[3]), 3, cv::Scalar(255, 0, 0), -1);
	}

	if (lines.size() > 0) {
		avg_Xs = (sum_Xs / lines.size());
		avg_Ys = (sum_Ys / lines.size());
		avg_Xe = (sum_Xe / lines.size());
		avg_Ye = (sum_Ye / lines.size());
	}

	if (avg_Xe - avg_Xs != 0) {
		slope = (avg_Ye - avg_Ys) / (avg_Xe - avg_Xs);
		constant = avg_Ys - slope * avg_Xs;
	}

	cv::line(input, cv::Point((1 - constant) / slope, 1), cv::Point((input.rows - constant) / slope, input.rows), cv::Scalar(255, 0, 255), 4, 0);
	cv::line(input, cv::Point(avg_Xs, avg_Ys), cv::Point(avg_Xe, avg_Ye), cv::Scalar(0, 255, 0), 4, 0);

	cv::Point fre = cv::Point(((((input.rows * 2) / 3) - constant) / slope) , (input.rows * 2) / 3);
	circle(input, fre, 10, cv::Scalar(255, 0, 0), -1);
	cv::circle(input, cv::Point(avg_Xs, avg_Ys), 5, cv::Scalar(0, 0, 0), -1);
	cv::circle(input, cv::Point(avg_Xe, avg_Ye), 5, cv::Scalar(0, 0, 0), -1);

	D = ((((input.rows * 2) / 3) - constant) / slope)- input.cols / 2;
	delta_D = D - D_desired ;


  if (delta_D>=-input.rows&&delta_D<=input.rows) {
    //ros::Publisher
  	ROS_INFO("distance:%d ",delta_D);
  }
	cv::Point TrackPoint = cv::Point(input.cols / 2 + D_desired, (input.rows * 2) / 3);
	cv::circle(input, TrackPoint, 10, cv::Scalar(0, 128, 255), -1);

  cv::imshow("input",input);

}

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
  lines(frame);
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
