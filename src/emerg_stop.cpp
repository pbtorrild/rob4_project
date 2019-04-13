#include <ros/ros.h>
#include <rob4_project/ClosestObj.h>
#include <geometry_msgs/Twist.h>
class movement{
  public:
    //Class constructor
    movement(const ros::Publisher& pub){
      ros::Publisher pub_=pub; //we define the publicher as the publisher send with the class defenition
    };

    //Consrtuct other functions:
    void callback_closest_obj(const rob4_project::ClosestObj::ConstPtr& reseved_data){
      float dist2obj=reseved_data->distance;
      float dist_th=0.4;

      geometry_msgs::Twist send_data;
      send_data.linear.x=0.1;

      ros::Rate loop_rate(100);
      do {
        pub_.publish(send_data);
        ros::spinOnce();
        loop_rate.sleep();
      } while(dist2obj>dist_th);

      send_data.linear.x=0;
      pub_.publish(send_data);
    }
   private:
     ros::Publisher pub_;
};
int main(int argc, char**argv){
  //standart ros initialize
  ros::init(argc, argv, "movement_pub");
  ros::NodeHandle nh;
  //define publisher
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);
  //create class instance monitor
  movement monitor(pub);
  //subscribe to the topic closest_object with que size 100 and
  //go to fucktion callback_closest_obj in the class movement with instance monitor
  ros::Subscriber sub = nh.subscribe("closest_object",100,&movement::callback_closest_obj,&monitor);
  ros::spin();
}
