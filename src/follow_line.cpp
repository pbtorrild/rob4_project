#include <ros/ros.h>
#include <iostream>
#include <cmath>

#include "sensor_msgs/image_encodings.h"
#include <image_transport/image_transport.h>
#include <torrilds_package/LineDist.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

cv::Mat input, frame, threshold1;
double distance_in;
void line_pub(ros::NodeHandle nh,ros::Publisher pub){
	//check for changes in emerg_stop status
	torrilds_package::LineDist send_data;
	send_data.line_dist=distance_in;
	pub.publish(send_data);
}

double Lines(cv::Mat& im) {

	//converts it to grayscale
	cv::cvtColor(im, threshold1, cv::COLOR_BGR2GRAY);

	//thresholds out only black and dark gray colors, 70 is current treshold,  set it lower and it will detect only darker black
	cv::threshold(threshold1, threshold1, 70, 255, cv::THRESH_BINARY_INV);

	//Converts image format back to BRG, otherwise other functions dont work
	cv::cvtColor(threshold1, threshold1, cv::COLOR_GRAY2BGR);

	//some variables to do some maths

	float sumx = 0;
	int g = 0;
	double trackx;
	int yvalue = 0;

	//checks top row on cutout picture from original full framme, to check different row on cutout picture, change the "yvalue" line up
	//checks if the line is present on each pixel from the thresholded picture, if yes adds the distance from left corner to that pixel

	for (int x = 0; x < im.cols &&ros::ok(); x++) {

		if(threshold1.at<cv::Vec3b>(cv::Point(x, yvalue))[0] > 1)
			{
    		sumx = sumx + x;
    		g = g+1;
		  }
	}
	//finds average distance to those pixels, a.k.a middle of the line
	trackx = sumx / g;
	//returns distance, again the one from the cutout picture
	return trackx;
}


void imageCallback(const sensor_msgs::ImageConstPtr msg)
{
  cv::Mat threshold1;
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
  input = cv_ptr->image;
  // here you select the area you want to chceck for finding the line, be carefull, when selecting the entire image,
  //then it will check only the top row later in algorythm,
  //to change that change the y-value, unit vector of y points down from top left corner btw
	cv::Rect inputRect = cv::Rect((input.cols * 2) / 3, (input.rows * 2) / 3, input.cols / 3, input.rows / 3);

	//cuts out the desired area
	cv::Mat frame = input(inputRect);
	// runs the algorythm as well as calculating the distance from left corner of the original non cut frame
	double distance = Lines(frame) + input.cols/3*2;

	// slaps a point on the original image so you can see if it works
  //show referal points
  cv::Point refereal_point = cv::Point(distance, input.rows*2/3);
	cv::circle(input, refereal_point, 8, cv::Scalar(255, 255, 255), -1);

	cv::Point TrackPoint1 = cv::Point(distance, input.rows*2/3);
	cv::circle(input, TrackPoint1, 8, cv::Scalar(0, 0, 255), -1);

  // shows the selected area for finding the line
  cv::rectangle(input, inputRect, cv::Scalar(255, 255, 255), 2, 8, 0);


	 //shows the original img with the dot on top of the line hopefully
	cv::imshow("view", input);
  cv::waitKey(30);

  ROS_INFO("%f",distance);
	distance_in=distance;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<torrilds_package::LineDist>("line_dist", 1);

  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 1, imageCallback);

	ros::Rate rate(15.);
	while (ros::ok()) {
		line_pub(nh,pub);
		rate.sleep();
		ros::spinOnce();
	}


  cv::destroyWindow("view");
}
