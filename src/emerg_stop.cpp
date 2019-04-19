#include <ros/ros.h>
#include <rob4_project/ClosestObj.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
float dist2obj=0;
void movement(float dist_th,ros::NodeHandle nh,ros::Publisher pub){
  geometry_msgs::Twist send_data;
  do {
    send_data.linear.x=0.1;
    pub.publish(send_data);
    ros::spinOnce();
  } while(dist2obj>=dist_th);

  send_data.linear.x=0;
  pub.publish(send_data);
}
//callback:
void callback_closest_obj(const rob4_project::ClosestObj::ConstPtr& reseved_data){
  dist2obj=reseved_data->distance;
}

int main(int argc, char**argv){
  //standart ros initialize
  ros::init(argc, argv, "movement_pub");
  ros::NodeHandle nh;
  //define publisher
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);
  //subscribe to the topic closest_object with que size 100 and
  //go to fucktion callback_closest_obj in the class movement with instance monitor
  ros::Subscriber sub_closest_object = nh.subscribe("closest_object",100,callback_closest_obj);
  while (ros::ok()) {
    movement(0.3,nh,pub);
  }
  ros::spin();
}
