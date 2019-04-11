#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char**argv){
  ros::init(argc, argv, "publish_velocity");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  ros::Rate rate(2);
  while(ros::ok()){
    geometry_msgs::Twist msg;
    msg.linear.x = 0.1;
    msg.angular.z =0.2;

    pub.publish(msg);
    rate.sleep();
  }
}
