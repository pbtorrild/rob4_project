#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h" //must be included since the data from the LaserScan is a Float32MultiArray
#include "sensor_msgs/LaserScan.h"
#include "torrilds_package/ClosestObj.h"

class lidar_info{
protected:  double turtlebot_width=0.195; //m
            double pi=3.14159265358979323846;
public:
  lidar_info(const ros::Publisher& closest_object_pub)
    : closest_object_pub_(closest_object_pub){

    }

  void callback_lidar_data(const sensor_msgs::LaserScan::ConstPtr& msg){
    //extracting information on the sensor message
    double scan_time = msg->scan_time;
    double range_max = msg->range_max;
    double range_min = msg->range_min;
    double angle_increment = msg->angle_increment;
    double angle_min = msg->angle_min;
    double ranges_size = msg->ranges.size(); // function in "std_msgs/Float32MultiArray.h"

    //finding coordinates of the input data
    double Range;
    double Angle;

    double obj_front_dist =range_max;
    double obj_front_angle;
    for (size_t i = 0; i <= ranges_size; i++){
      //Polar coordinates
      Range = msg->ranges[i];
      Angle = (angle_min + (i* angle_increment));

      //Cartisian coordinates
      double x = Range*cos(Angle);
      double y = Range*sin(Angle);

      //Find closest opbject
      if(Range<obj_front_dist&&Range>range_min){
        obj_front_dist=Range;
        obj_front_angle=Angle;
      }
    }
    //publish the closest opbject
    torrilds_package::ClosestObj send_data;
    send_data.forward_obj.distance=obj_front_dist;
    send_data.forward_obj.angle=obj_front_angle;
    closest_object_pub_.publish(send_data);
  }
private:
  ros::Publisher closest_object_pub_;
};



int main(int argc, char **argv)
{
  //initialize subsciber
  ros::init(argc, argv, "laser_info");
  ros::NodeHandle n;
  ros::Publisher closest_obj=n.advertise<torrilds_package::ClosestObj>("closest_object",10);
  lidar_info monitor(closest_obj);
  //sibscribe to the scan topic and go to the function above
  ros::Subscriber sub = n.subscribe("scan", 1000, &lidar_info::callback_lidar_data,&monitor);
  //spin untill there is no more rosmaster (roscore)
  ros::spin();


  return 0;
}
