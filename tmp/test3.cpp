#include "ros/ros.h"
#include "pr2_controllers_msgs/JointTrajectoryControllerState.h"
#include <boost/thread.hpp>

double start=0.0;
double count=0.0;

void callback(const pr2_controllers_msgs::JointTrajectoryControllerStateConstPtr& msg){
  count+=1.0;
  ROS_INFO("Hz=%f", count/(ros::Time::now().toSec()-start));
}

int main(int argc, char ** argv){
  ros::init(argc, argv, "clientnodehz");
  ros::NodeHandle n;
  while(start==0.0){
    start=ros::Time::now().toSec();
  }
  ros::Subscriber sub = n.subscribe<pr2_controllers_msgs::JointTrajectoryControllerState>("/r_arm_controller/state", 10, callback);
  ros::spin();
  return 0;
}
