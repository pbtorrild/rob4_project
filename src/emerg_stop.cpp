#include <ros/ros.h>
#include <torrilds_package/ClosestObj.h>
#include <torrilds_package/EmergStop.h>
class emerg_status{
private:
  // distance threshold
  float dist_th=0.3;
protected:
  bool emerg_stop=true;
  bool published_emerg_stop=true;

  bool emerg_speed_up=true;
  bool published_emerg_speed_up;
public:
  //funcktions below
  void emerg_pub(ros::NodeHandle nh,ros::Publisher pub){
    //check for changes in emerg_stop status
    torrilds_package::EmergStop send_data;
    if(published_emerg_stop!=emerg_stop){
      send_data.emerg_stop=emerg_stop;
    }
    if(published_emerg_stop!=emerg_stop){
      send_data.emerg_speed_up=emerg_speed_up;
    }
    pub.publish(send_data);
  }
  //callback:
  void callback_closest_obj(const torrilds_package::ClosestObj::ConstPtr& reseved_data){
    float forward_dist=reseved_data->forward_obj.distance;
    if (forward_dist <= dist_th){
      emerg_stop=true;
    }
    else{
      emerg_stop=false;
    }
    float back_dist=reseved_data->backward_obj.distance;
    if (back_dist <= dist_th){
      emerg_speed_up=true;
    }
    else{
      emerg_speed_up=false;
    }
  }
  void callback_emerg_stop(const torrilds_package::EmergStop::ConstPtr& reseved_data){
    published_emerg_stop=reseved_data->emerg_stop;
    published_emerg_speed_up=reseved_data->emerg_speed_up;
  }
};

int main(int argc, char**argv){
  //standart ros initialize
  ros::init(argc, argv, "emerg_stop");
  ros::NodeHandle nh;
  //define publisher
  ros::Publisher pub = nh.advertise<torrilds_package::EmergStop>("emerg_stop_status", 10);
  //set class member
  emerg_status monitor;
  //subscribe to the topic closest_object with que size 100 and
  //go to fucktion callback_closest_obj in the class movement with member &monitor
  // plz note that this isnt monitor but a constant verson of thet member
  ros::Subscriber sub_closest_object = nh.subscribe("closest_object",100,&emerg_status::callback_closest_obj,&monitor);
  ros::Subscriber sub_emerg_stop = nh.subscribe("emerg_stop_status",100,&emerg_status::callback_emerg_stop,&monitor);

  //initialise the node with an emerg_stop
  while (ros::ok()) {
    monitor.emerg_pub(nh,pub);
    ros::spinOnce();
  }

}
