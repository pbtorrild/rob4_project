#include "ros/ros.h"
#include "rob4_project/ClosestObj.h"
#include "geometry_msgs/Twist.h"

class movements{
public:
  movements(const ros::Publisher& move_pub)
    : movement_pub_(move_pub){

    }
  void emerg_stop(){
    geometry_msgs::Twist send_data;
    send_data.linear.x=0;
    send_data.linear.y=0;
    send_data.linear.z=0;

    send_data.angular.x=0;
    send_data.angular.y=0;
    send_data.angular.z=0;
    movement_pub_.publish(send_data);
  }
  void forward(){
    geometry_msgs::Twist send_data;
    send_data.linear.x=0.2;
    send_data.linear.y=0;
    send_data.linear.z=0;

    send_data.angular.x=0;
    send_data.angular.y=0;
    send_data.angular.z=0;
    movement_pub_.publish(send_data);
  }
  void callback_closest_obj(const rob4_project::ClosestObj::ConstPtr& msg){
      double dist=msg->distance;
      if (dist<=0.3) {
        movements::emerg_stop();
      }
      else{
        movements::forward();
      }
    }
private:
  ros::Publisher movement_pub_;
};

int main(int argc, char **argv)
{
  //initialize subsciber
  ros::init(argc, argv, "movement_override");
  ros::NodeHandle n;
  ros::Publisher movement_pub=n.advertise<geometry_msgs::Twist>("cmd_vel",10);
  movements monitor(movement_pub);
  //sibscribe to the scan topic and go to the function above
  ros::Subscriber sub = n.subscribe("scan", 1000, &movements::callback_closest_obj,&monitor);
  //spin untill there is no more rosmaster (roscore)
  ros::spin();


  return 0;
}
