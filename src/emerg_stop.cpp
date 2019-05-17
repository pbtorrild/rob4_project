#include <ros/ros.h>
#include <torrilds_package/ClosestObj.h>
#include <torrilds_package/EmergStop.h>
class emerg_status{
private:
  // distance threshold
  float dist_th=0.3;
protected:
  bool emerg_stop=true;
  bool emerg_speed_up=false;
public:
  float scan_time;
  //funcktions below
  void emerg_pub(ros::NodeHandle nh,ros::Publisher pub){
    //check for changes in emerg_stop status
    torrilds_package::EmergStop send_data;

    send_data.emerg_stop=emerg_stop;
    send_data.emerg_speed_up=emerg_speed_up;

    pub.publish(send_data);
  }
  //callback:
  void callback_closest_obj(const torrilds_package::ClosestObj::ConstPtr& reseved_data){
    float forward_dist=reseved_data->forward_obj.distance;
    float back_dist=reseved_data->backward_obj.distance;
    scan_time=reseved_data->scan_time;
    ROS_INFO("%f : %f",forward_dist,back_dist);
    //test if the closest object ahead is over or under the threshold
    if (forward_dist <= dist_th){
      emerg_stop=true;
    }
    else{
      emerg_stop=false;
    }
    //test if the closest object behind is over or under the threshold
    if (back_dist <= dist_th && emerg_stop!=true){
      emerg_speed_up=true;
    }
    else{
      emerg_speed_up=false;
    }
  }
};

int main(int argc, char**argv){
  //standart ros initialize
  ros::init(argc, argv, "emerg_stop");
  ros::NodeHandle nh;
  //define publisher
  ros::Publisher pub = nh.advertise<torrilds_package::EmergStop>("emerg_stop_status", 1);
  //set class member
  emerg_status monitor;
  //subscribe to the topic closest_object with que size 100 and
  //go to fucktion callback_closest_obj in the class movement with member &monitor
  // plz note that this isnt monitor but a constant verson of thet member

  ros::Subscriber sub_closest_object = nh.subscribe("closest_object",1,&emerg_status::callback_closest_obj,&monitor);

  //initialise the node with an emerg_stop

  while (ros::ok()) {
    ros::Rate rate(monitor.scan_time);
    monitor.emerg_pub(nh,pub);
    rate.sleep();
    ros::spinOnce();
  }

}
