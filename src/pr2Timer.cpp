#include "pr2_timer/pr2Timer.h"

double pr2Timer::average(vector<double> times){

  double sum=0.0;
  for(int i = 0; i < times.size(); i++){
    sum+=times[i];
  }
  double ret = sum/(times.size());

  return ret;
}

double pr2Timer::std_deviation(vector<double> times){

  double avg=average(times);
  double sum=0.0;
  for(int i = 0; i < times.size();i++){
    double val=times[i]-avg;
    if(val<0)
      val*=-1;
    sum+=val;
  }
  double ret = sum/(times.size()-1);

  return ret;
}

vector<double> pr2Timer::removeOutliers(vector<double> times){

  double dev=std_deviation(times);
  double avg=average(times);
  vector<double> inliers;
  for(int i = 0;i<times.size();i++){
    if(!(((times[i]-avg)/dev)>1.96))
      inliers.push_back(times[i]);
  }

  return inliers;
}

void pr2Timer::resetDefAndPath(double def[], double path[]){
  def[0] = 0.0;
  def[1] = 0.0;
  def[2] = 0.0;
  def[3] = -0.15;
  def[4] = 0.0;
  def[5] = 0.0;
  def[6] = 0.0;

  path[0] = 0.0;
  path[1] = 0.0;
  path[2] = 0.0;
  path[3] = -0.15;
  path[4] = 0.0;
  path[5] = 0.0;
  path[6] = 0.0;
}

vector<double> pr2Timer::timePrimitive(double primitive){
  
  double def[]={0.0,0.0,0.0,-0.15,0.0,0.0,0.0};
  double path[]={0.0,0.0,0.0,-0.15,0.0,0.0,0.0};
  vector<double> joint_times;
  vector<vector<double> > collective_times;
  vector<double> averages;
  if(primitive == 0.0){
    for(int i = 0; i < 7;i++)
      averages.push_back(0.0);
    return averages;
  }
  for(int i = 0; i < 7; i++){
    double delta0=rarm->min[i];
    double delta1=delta0+primitive;
    if(primitive<(rarm->max[i]-rarm->min[i])){
      for(;delta1<rarm->max[i];delta0=delta1, delta1+=primitive){
	def[i]= delta0; path[i]=delta1;
	joint_times.push_back(rarm->time(def, path, 0));
      }
    }
    else
      joint_times.push_back(-1.0);
      
    collective_times.push_back(joint_times);
    joint_times.clear();
    resetDefAndPath(def, path);
  }
  vector<double> temp;
  for(int i = 0; i < collective_times.size(); i++){
    
    //cout<<collective_times[i].size()<<endl;
    temp = removeOutliers(collective_times[i]);
    
    collective_times[i] = temp;
    averages.push_back(average(collective_times[i]));
    
  }
  cout<<endl; 
  return averages;
}


void pr2Timer::timeStep(double step, char * filename){
  ROS_INFO("You got to the  timestep");
  vector<double> averages;
  ofstream file(filename);
  vector<vector<double> > collective_averages;
  for(double primitive = 0.0; primitive < 6.28; primitive+=step){
    averages = timePrimitive(primitive);
    collective_averages.push_back(averages);
    cout<<"Collective averages size"<<collective_averages.size()<<endl;
  }
  file<<collective_averages.size()<<" "<<step<<endl;
  for(int i = 0; i < collective_averages.size();i++){
    //cout<<(i+1)*step<<endl;
    for(int j = 0; j < 7; j++){
      //cout<<collective_averages[i][j]<<" ";
      file<<collective_averages[i][j]<<" ";
    }
    file<<endl;
    //cout<<endl;
  }
  file.close();
}

pr2Timer::pr2Timer(){}
pr2Timer::pr2Timer(Arm * arm){
  rarm = arm;
}



void pr2Timer::parseTimeFile(char * filename){
  ifstream file(filename);
  int num_steps;
  double step_size, temp;
  file>>num_steps>>step_size;
  vector<double> temp_times;
  step=step_size;
  for(int i = 0; i < num_steps;i++){
    for(int j = 0; j<7;j++){
      file>>temp;
      temp_times.push_back(temp);
    }
    combined_joint_times.push_back(temp_times);
    temp_times.clear();
  }
}

/*The time it takes to move a joint a certain distance*/
/*preconditions: distance>=0.0&&distance<(2*pi-step)   */
double pr2Timer::getTimeForSingleJoint(double distance, int joint){
  if(distance<0.0||distance > (2*3.1415926-step)){
    ROS_ERROR("The joint cannot move that range");
    exit(-1);
  }
  int index = (distance/step);
  if(combined_joint_times[index][joint]==-1){
    ROS_ERROR("The step used to generate the time file does not have enought granularity to determing the time for joint %d to move %f radians", joint, distance);
  }
  cout<<"Distance is "<< distance<<" and joint is "<<joint<<" and [index] and [index+1] is ["<<combined_joint_times[index][joint]<<"] ["<<combined_joint_times[index+1][joint]<<"] "<<endl;
  double time = (combined_joint_times[index][joint])+distance*(combined_joint_times[index+1][joint]-combined_joint_times[index][joint])/step;
  return time;
}

double pr2Timer::getMotionTime(std::vector<double> &start, std::vector<double> &end){
  if(combined_joint_times.size()==0){
    ROS_ERROR("You have to load a time file before getting motion times");
    exit(-1);
  }
  double max = 0.0;
  for(int i = 0; i < 7; i++){
    double distance = end[i]-start[i];
    if(distance < 0)
      distance *=-1;
    double temp_time = getTimeForSingleJoint(distance, i);
    if(temp_time>max)
      max = temp_time;
  }
  return max;
}
