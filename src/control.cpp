#include <ros/ros.h>

#include "geometry_msgs/Twist.h"

#include <torrilds_package/ClosestObj.h>
#include <torrilds_package/EmergStop.h>
#include <torrilds_package/LineDist.h>
#include <torrilds_package/SignsFound.h>
class control_data{
private:
  //distance in pixel to line
  float desired_pix_dist=320;
  // vel
  float vel_30=0.01;
  float std_vel =0.03;
  float vel_50 = 0.03;
  float vel_70 =0.05;
  float speed_up_vel=0.05;
protected:
  //emergcy stop status
  bool emerg_stop=true;
  bool emerg_speed_up=false;
  //sign input
  bool MainSideRoad=false;
  bool Yield=false;
  bool Kids=false;
  bool DontGoLeft=false;
  bool Seventy=false;
  bool Thirty=false;
  bool Fifty=false;
  bool BothWaysNo=false;
  bool Cross=false;
  //humans and car detection
  bool human=false;
  bool car=false;
  //change of road
  float line_dist_px=0;
public:
  //funcktions below
  double get_angular_vel(double distance_in){
    double angular_vel;
    //computiation
    double dist_2l_error=distance_in-desired_pix_dist;
    ROS_INFO("Error: %f",dist_2l_error);
      //we determine k by taking the maximum wanted angular vel and diviting it by the maximum error
      float k=1/320; //A_vel_max/e_max , A_vel_max=0.5 & e_max=320
      angular_vel=-dist_2l_error*k;
    return angular_vel;
  }
  void cmd_vel(ros::NodeHandle nh,ros::Publisher pub){
    //check for changes in emerg_stop status
    geometry_msgs::Twist send_data;
    //control funcktions
    //fist check for emergency stop
    if (emerg_stop!=true) {
      send_data.linear.x=std_vel; // go with a std vel of 50 km/h

      //sign to determine speed
      if (Seventy==true) {
        send_data.linear.x=vel_70;
      }
      if (Fifty==true) {
        send_data.linear.x=vel_50;
      }
      if (emerg_speed_up==true) {
        send_data.linear.x=speed_up_vel;
      }
      //////////////////////////////
      //Do the standard angular change
      send_data.angular.z=get_angular_vel(line_dist_px);

      //////////////////////////////
      // Special cases of change in vel and angle

      //perfon a stop if there is a human at a crossing
      if (human==true && Cross==true) {
        send_data.linear.x=0;
        send_data.angular.z=get_angular_vel(line_dist_px);
      }

    }
    else{
      send_data.linear.x=0;
      send_data.angular.z=0;
    }
    pub.publish(send_data);
  }
  void end_of_alg(ros::NodeHandle nh,ros::Publisher pub){

    geometry_msgs::Twist send_data;

    send_data.linear.x=0;
    send_data.angular.z=0;

    pub.publish(send_data);
    ros::spinOnce();
  }
  //callback:
  void callback_signs_found(const torrilds_package::SignsFound::ConstPtr& reseved_data){
    MainSideRoad=reseved_data->MainSideRoad;
    Yield=reseved_data->Yield;
    Kids=reseved_data->Kids;
    DontGoLeft=reseved_data->DontGoLeft;
    Seventy=reseved_data->Seventy;
    Thirty=reseved_data->Thirty;
    Fifty=reseved_data->Fifty;
    BothWaysNo=reseved_data->BothWaysNo;
    Cross=reseved_data->Cross;

  }
  void callback_emerg_stop(const torrilds_package::EmergStop::ConstPtr& reseved_data){
    emerg_stop=reseved_data->emerg_stop;
    emerg_speed_up=reseved_data->emerg_speed_up;
  }
  void callback_road_change(const torrilds_package::LineDist::ConstPtr& reseved_data){
    line_dist_px=reseved_data->line_dist;
  }
};

int main(int argc, char**argv){
  //standart ros initialize
  ros::init(argc, argv, "vel_controller");
  ros::NodeHandle nh;
  //define publisher
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  //set class member
  control_data monitor;
  //subscibers
  ros::Subscriber sub_signs = nh.subscribe("signs_found",1,&control_data::callback_signs_found,&monitor);
  ros::Subscriber sub_emerg_stop = nh.subscribe("emerg_stop_status",1,&control_data::callback_emerg_stop,&monitor);
  ros::Subscriber sub_road_change = nh.subscribe("line_dist",1,&control_data::callback_road_change,&monitor);
  //initialise the node with an emerg_stop
  ros::Rate rate(15.);
  while (ros::ok()) {
    monitor.cmd_vel(nh,pub);
    rate.sleep();
    ros::spinOnce();
  }
  monitor.end_of_alg(nh,pub);
}
