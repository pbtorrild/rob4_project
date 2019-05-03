#include <ros/ros.h>

#include "geometry_msgs/Twist.h"

#include <torrilds_package/ClosestObj.h>
#include <torrilds_package/EmergStop.h>
#include <torrilds_package/RoadChange.h>
#include <torrilds_package/SignsFound.h>
class control_data{
private:
  // distance threshold
  float dist_th=0.3;
  // vel
  float std_vel =0.5;
  float speed_up_vel=0.7;
protected:
  //emergcy stop status
  bool emerg_stop=true;
  bool emerg_speed_up=false;
  //sign input
  bool speed70;
  bool stop_sign;
  //change of road
  float rot_vel=0;
public:
  //funcktions below
  void cmd_vel(ros::NodeHandle nh,ros::Publisher pub){
    //check for changes in emerg_stop status
    geometry_msgs::Twist send_data;
    //control funcktions
    //fist check for emergency stop
    if (emerg_stop!=true) {
      //sign input
      if (speed70=true) {
        send_data.linear.x=0.7;
      } else if (stop_sign=true) {
        send_data.linear.x=0;
      } else {
        /* code  for no signs */
        send_data.linear.x=std_vel;
      }
      //////////////////////////////
      if (emerg_speed_up=true) {
        send_data.linear.x=speed_up_vel;
      }
      //Do the angular change
      send_data.angular.z=rot_vel;
    }
    else{
      send_data.linear.x=0;
      send_data.angular.z=0;
    }
    pub.publish(send_data);
  }
  //callback:
  void callback_signs_found(const torrilds_package::SignsFound::ConstPtr& reseved_data){
    speed70=reseved_data->speed70;
    stop_sign=reseved_data->stop_sign;
  }
  void callback_emerg_stop(const torrilds_package::EmergStop::ConstPtr& reseved_data){
    emerg_stop=reseved_data->emerg_stop;
  }
  void callback_road_change(const torrilds_package::RoadChange::ConstPtr& reseved_data){
    rot_vel=reseved_data->rot_vel;
  }
};

int main(int argc, char**argv){
  //standart ros initialize
  ros::init(argc, argv, "vel_controller");
  ros::NodeHandle nh;
  //define publisher
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  //set class member
  control_data monitor;
  //subscibers
  ros::Subscriber sub_signs = nh.subscribe("signs_found",100,&control_data::callback_signs_found,&monitor);
  ros::Subscriber sub_emerg_stop = nh.subscribe("emerg_stop_status",100,&control_data::callback_emerg_stop,&monitor);
  ros::Subscriber sub_road_change = nh.subscribe("road_status",100,&control_data::callback_road_change,&monitor);
  //initialise the node with an emerg_stop
  while (ros::ok()) {
    monitor.cmd_vel(nh,pub);
    ros::spinOnce();
  }

}
