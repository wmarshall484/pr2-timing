#include <trajectory_recorder/arm.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>

using namespace std;

void spinUntilWithinError(){
  pr2_controllers_msgs::JointTrajectoryControllerStateConstPtr state = ros::topic::waitForMessage<pr2_controllers_msgs::JointTrajectoryControllerState>("/r_arm_controller/state");
  double sum=0.0;
  while(sum>=0.0){
    for(int i = 0; i < 7;i++){
      sum+=((*state).error.positions[i])*((*state).error.positions[i]); 
    }
    ROS_INFO("The error in the position is %f", sum);
    sum=0.0;
    state = ros::topic::waitForMessage<pr2_controllers_msgs::JointTrajectoryControllerState>("/r_arm_controller/state");
  }
}

int main(int argc, char ** argv){
  ros::init(argc, argv, "move_arm");
  Arm arm("right");
  double pose[]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
  double t1 = ros::Time::now().toSec();
  arm.sendArmToConfiguration(pose, 2.0);
  spinUntilWithinError();
  double duration = ros::Time::now().toSec()-t1;
  ROS_INFO("The command took %f seconds to send", duration);
  return 0;
}
