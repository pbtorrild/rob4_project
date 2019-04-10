#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h" //must be included since the data from the LaserScan is a Float32MultiArray
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

class Movement{
private:

public:
  void emerg_stop(const geometry_msgs::Twist::ConstPtr msg);
};
void emerg_stop(const geometry_msgs::Twist::ConstPtr msg){
  ROS_WARN("Emergency stop enabled");
  msg.linear.x=0;
  msg.linear.y=0;
  msg.linear.z=0;

  msg.angular.x=0;
  msg.angular.y=0;
  msg.angular.z=0;
  pub.publish(msg);
}
