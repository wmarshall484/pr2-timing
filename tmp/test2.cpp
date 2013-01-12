#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>

using namespace std;

int main(int argc, char ** argv){
  double count=0.0;
  ros::init(argc, argv, "hznode");
  ros::NodeHandle n;
  pr2_controllers_msgs::JointTrajectoryControllerStateConstPtr state;
  double start=0.0;
  while(start==0.0)
    start = ros::Time::now().toSec();
  
  double elapsed =0.0;
    while(1){
      count+=1;
      elapsed = (ros::Time::now().toSec())-start;
      state = ros::topic::waitForMessage<pr2_controllers_msgs::JointTrajectoryControllerState>("/r_arm_controller/state");
      ROS_INFO("Hz = %f count = %f elapsed = %f start = %f now = %f", count/elapsed, count, elapsed, start, ros::Time::now().toSec());
    }
  return 0;
}
