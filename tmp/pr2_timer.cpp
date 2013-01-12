#include "ros/ros.h"
#include "pr2_controllers_msgs/JointTrajectoryControllerState.h"
#include "trajectory_recorder/arm.h"

#define primitive 0.13962634016

int main(int argc, char** argv){
  ros::init(argc, argv, "timer_node");
  ros::Subscriber sub;
  Arm a("right", sub);
  double initial[] = {0.00012441893679504545, 0.015836695675442769, 0.0058693130576603636, -0.43427709917664892, 0.00041232768130683439, -0.74775342903554254, 0.00048162289359598986};
  double final[]={-0.5661, 0.3443, -1.2206, -1.7038, 3.6892, -1.3877, -3.8171};
  double time = a.time(initial, final, 2);
  return 0;
}
