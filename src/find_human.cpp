#include <ros/ros.h>
#include <iostream>
#include <cmath>

#include "sensor_msgs/image_encodings.h"
#include <image_transport/image_transport.h>
#include <rob4_pkg/RoadObj.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
bool human_found;

void road_obj_pub(ros::NodeHandle nh,ros::Publisher pub){
	//check for changes in emerg_stop status
	rob4_pkg::RoadObj send_data;
	//threshold for the dot to be inside the view_line of the cam
	send_data.human_found=human_found;
	pub.publish(send_data);
}

//use non-maxima suppression to eliminate multiple overlapping boxes
void find_human(cv::Mat img_re)
{
	//initalise the Histogram of oriented gradients "file"..."type"..thingy.
	cv::HOGDescriptor HogFile;

	//inside the HOGDescriptor in opencv theres a function called "setSVMDetector", it is used to detect humans "Returns coefficients of the classifier trained for people detection (for 64x128 windows)."
	HogFile.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector()); //theres an alternative called "getDaimlerPeopleDetector ()" , but couldnt get it to work

	//Make vector of opencvRectangles and call them found, then make another called found_filtered...
	std::vector<cv::Rect> BoundingBoxRect, BoundingBoxRect_filtered;
	//running the detector on fefault settings, puts
	//Syntax: CascadeClassifier::detectMultiScale(const Mat& image, vector<Rect>& objects, double scaleFactor=1.1, int minNeighbors=3, int flags=0, Size minSize=Size(), Size maxSize=Size())
	HogFile.detectMultiScale(img_re, BoundingBoxRect, 0, cv::Size(8, 8), cv::Size(32, 32), 1.05, 2);

	//size_t is an unsigned integer array commonly used for array indexing and loop counting... make one called i and another called j... duh
	size_t i, j;

	if (BoundingBoxRect.size()>0) {
		human_found=true;
	}
	else{human_found=false;}


	//The Detector returns a slightly too large bounding box. this next two forloops rescales/shrinks it to match better.
	for (i = 0; i < BoundingBoxRect.size(); i++)
	{
		//initialize a rectangle called r and specify it as bieng equal to the BoundingBox rectangle
		cv::Rect r = BoundingBoxRect[i];

		//Dont put small detections inside big detections.
		for (j = 0; j < BoundingBoxRect.size(); j++)
			if (j != i && (r & BoundingBoxRect[j]) == r)
				break;
		if (j == BoundingBoxRect.size())
			BoundingBoxRect_filtered.push_back(r); //push_back puts
	}
	//The Detector returns a slightly too large bounding box. this next forloop rescales/shrinks it to match better.
	for (i = 0; i < BoundingBoxRect_filtered.size(); i++)
	{
		cv::Rect r = BoundingBoxRect_filtered[i];
		r.x += cvRound(r.width*0.1); //cvRound rounds floating-point number to the nearest integer, and the insides downscale by whats written.
			r.width = cvRound(r.width*0.8);
		r.y += cvRound(r.height*0.06);
		r.height = cvRound(r.height*0.9);
		cv::rectangle(img_re, r.tl(), r.br(), cv::Scalar(0, 255, 0), 2);
	}
	//display the results
	cv::imshow("view_human", img_re);
  cv::waitKey(10);
}
void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg)
{
	cv::Mat input;
  try
  {
    input = cv::imdecode(cv::Mat(msg->data),1);//convert compressed image data to cv::Mat

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert to image!");
  }
	find_human(input);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	cv::namedWindow("view_human");
  cv::startWindowThread();
	ros::Subscriber sub = nh.subscribe("/usb_cam_1/main_cam/image_raw/compressed", 1, imageCallback);
	ros::Publisher pub = nh.advertise<rob4_pkg::RoadObj>("road_obj", 1);
  while (ros::ok()) {
		road_obj_pub(nh,pub);
		ros::spinOnce();
	}

  cv::destroyWindow("view_human");
}
