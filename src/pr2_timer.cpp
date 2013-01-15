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
  for(int i = 0; i < 7; i++){
    if(primitive<(rarm->max[i]-rarm->min[i])){
      double delta0=rarm->min[i];
      double delta1=delta0+primitive;
      int count = 0;
      while(count<100){
	for(;delta1<rarm->max[i];delta0=delta1, delta1+=primitive){
	  def[i]= delta0; path[i]=delta1;
	  joint_times.push_back(rarm->time(def, path, 0));
	  count++;
	}
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
  vector<double> averages;
  ofstream file(filename);
  vector<vector<double> > collective_averages;
  for(double primitive = step; primitive < 6.28; primitive+=step){
    averages = timePrimitive(primitive);
    collective_averages.push_back(averages);
  }
  file<<collective_averages.size()<<" "<<step<<endl;
  for(int i = 0; i < collective_averages.size();i++){
    cout<<(i+1)*step<<endl;
    for(int j = 0; j < 7; j++){
      cout<<collective_averages[i][j]<<" ";
      file<<collective_averages[i][j]<<" ";
    }
    file<<endl;
    cout<<endl;
  }
}

pr2Timer::pr2Timer(Arm * arm){
  rarm = arm;
}
