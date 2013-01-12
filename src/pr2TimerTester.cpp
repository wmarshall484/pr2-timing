#include "pr2_timer/pr2Timer.h"

int main(int argc, char ** argv){
  ros::init(argc, argv, "timernode");
  ros::Subscriber sub;
  Arm rarm("right", sub);
  double step = 0.07;
  pr2Timer timer(&rarm);
  timer.timeStep(step, "/home/wmarshall/workspace/pr2_timer/times.txt");
  //timer.parseTimeFile("/home/wmarshall/workspace/pr2_timer/times.txt");
  vector<double> start, end;
  /*for(int i = 0; i < 7; i++){
    start.push_back(0.0);
    end.push_back(0.0);
  }
  cout<<"That motion takes "<<timer.getMotionTime(start, end)<<" seconds"<<endl;*/
  
  cout<<timer.combined_joint_times.size()<<" "<<timer.step<<endl;
  for(int i = 0; i < timer.combined_joint_times.size();i++){
    for(int j = 0; j < 7; j++){
      cout<<timer.combined_joint_times[i][j]<<" ";
    }
    cout<<endl;
    } 
  ros::shutdown();
  
  return 0;
}
