#ifndef PR2TIME
#define PR2TIME
#include "ros/ros.h"
#include "trajectory_recorder/arm.h"
#include <vector>
#include <fstream>

class pr2Timer{
 public:
  pr2Timer(Arm *arm);
  pr2Timer();
  void timeStep(double step, char * filename);
  vector<double> timePrimitive(double primitive);
  void parseTimeFile(char * filename);
  double getMotionTime(std::vector<double> &start, std::vector<double> &end);
  vector<vector<double > > combined_joint_times;
  double step;
 private:
  double getTimeForSingleJoint(double distance, int joint);
  void resetDefAndPath(double def[], double path[]);
  vector<double> removeOutliers(vector<double> times);
  double std_deviation(vector<double> times);
  double average(vector<double> times);
  
  Arm *rarm;
  
  
};
#endif
