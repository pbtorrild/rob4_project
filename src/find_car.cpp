//Librarys duh
#include <ros/ros.h>
#include "sensor_msgs/image_encodings.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <math.h>

#include <rob4_pkg/RoadObj.h>
bool car_found;

void road_obj_pub(ros::NodeHandle nh,ros::Publisher pub){
	//check for changes in emerg_stop status
	rob4_pkg::RoadObj send_data;
	//threshold for the dot to be inside the view_line of the cam
	send_data.car_found=car_found;
	pub.publish(send_data);
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
	//initializing the imiges used for the processing stages
  cv::Mat img2, morph1, img;
	//As we want to look for the colour red, and we need the location of the range on the color wheel to be positive angles,
	//we invert the colour wheel by changing the location of red to blues position, and converting it to hsv
	//note, images in opencv are in bgr
	//img is input, img2 is output.
	cvtColor(input, img2, cv::COLOR_RGB2HSV);
	//here we initialize the variables to be used later. s=saturation, v=value
	int s1 = 0.35 * 255, s2 = 0.75 * 255, v1 = 0.20 * 255, v2 = 1 * 255;
	//inRange Thresholds for any color value inrange of the two points specified.
	//Scalar(Hue,Saturation,Value)
	inRange(img2, cv::Scalar(110, s1, v1), cv::Scalar(140, s2, v2), img2);

	//initialize the "elements" used in the morphology later, These define the shape and size of the kernals used.
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(13, 13));
	cv::Mat element2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 17));

	//Here we run the morphology.
	//SYNTAX: morpology(inputPic,OutputPic,TheMorpgologyType,Shape and size Of the kernal)
	cv::morphologyEx(img2, morph1, cv::MORPH_CLOSE, element);
	cv::morphologyEx(morph1, img2, cv::MORPH_OPEN, element2);

	//The Vector of VectorPoints is initialized
	std::vector<std::vector<cv::Point>> contours;
	//SYNTAX: findContours(inputPic,where to save the objects,Mode,Method)
	//RETR_EXTERNAL ensures bounding boxes isnt detected inside others
	//Here we find the Contours around any red lights
	cv::findContours(img2, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
  if (contours.size()==2) {
    //Initialize the rectangle array used for storing the Rearlight bounding boxes..
  	cv::Rect RedLight[2];

  	//Saves the Bounding boxes the redlight array.
  	for (int i = 0; i < contours.size(); i++) {
  		RedLight[i] = cv::boundingRect(contours[i]);
  	}
  	//If the first elements x-value, in Redlight's is less than the second. then the statement runs.
  	//this is to know the position of the lights in relation to each other
  	if (RedLight[0].x < RedLight[1].x) {
  		//The width of the car is equal to the distance between the two rear lights
  		int WidthOfCar = abs(RedLight[1].x - RedLight[0].x);
  		//initialize the "Width" variable used to mark the cars position with a bounding box.
  		cv::Point Width = cv::Point(RedLight[0].x, RedLight[0].y - (WidthOfCar / 3));
  		//Define the oposing corners of the bounding box
  		cv::Rect CarHere = cv::Rect(Width, cv::Point(Width.x + WidthOfCar + RedLight[1].width, Width.y + WidthOfCar));
  		//Make the bounding box placed around the car. with the color code of violet~ish and a thichness of 3 pixels
  		cv::rectangle(input, CarHere, CV_RGB(230, 0, 250), 3);
  	}
  	//Check if the other redlight is on the left side
  	else {
  		int WidthOfCar = abs(RedLight[1].x - RedLight[0].x);
  		cv::Point Width = cv::Point(RedLight[1].x, RedLight[1].y - (WidthOfCar / 3));
  		cv::Rect CarHere = cv::Rect(Width, cv::Point(Width.x + WidthOfCar + RedLight[0].width, Width.y + WidthOfCar));
  		cv::rectangle(input, CarHere, CV_RGB(230, 0, 250), 3);

  	}
    car_found=true;
  }
  else{car_found=false;}

  cv::imshow("view_car",input);
  cv::waitKey(10);
//int  Processing(input);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	cv::namedWindow("view_car");
  cv::startWindowThread();
	ros::Subscriber sub = nh.subscribe("/usb_cam_1/main_cam/image_raw/compressed", 1, imageCallback);

  ros::Publisher pub = nh.advertise<rob4_pkg::RoadObj>("road_obj", 1);
  while (ros::ok()) {
		road_obj_pub(nh,pub);
		ros::spinOnce();
	}

  cv::destroyWindow("view_car");
}
