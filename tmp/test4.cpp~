#include "ros/ros.h"
#include "pr2_controllers_msgs/QueryTrajectoryState.h"

int main(int argc, char ** argv){
  ros::init(argc, argv, "servicenodehz");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<pr2_controllers_msgs::QueryTrajectoryState>("/r_arm_controller/query_state", true);
  pr2_controllers_msgs::QueryTrajectoryState msg;
  while(ros::Time::now().toSec()==0.0);
  msg.request.time=ros::Time::now();
  double count =0.0;
  double start = ros::Time::now().toSec();
  while(1){
    count+=1.0;
    client.call(msg);
    //ROS_INFO("Hz=%f", count/(ros::Time::now().toSec()-start));
    ROS_INFO("The joint states are [%f, %f, %f, %f, %f, %f, %f]", msg.response.position[0], msg.response.position[1], msg.response.position[2], msg.response.position[3], msg.response.position[4], msg.response.position[5], msg.response.position[6]);
    msg.request.time=ros::Time::now();
  }
  return 0;
}
