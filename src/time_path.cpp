#include "ros/ros.h"
#include "trajectory_recorder/arm.h"
#include <vector>
#include <fstream>

double average(vector<double> times){
  
  double sum=0.0;
  for(int i = 0; i < times.size(); i++){
    sum+=times[i];
  }
  double ret = sum/(times.size());
  
  return ret;
}

double std_deviation(vector<double> times){
  
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

vector<double> removeOutliers(vector<double> times){
  
  double dev=std_deviation(times);
  double avg=average(times);
  vector<double> inliers;
  for(int i = 0;i<times.size();i++){
    if(!(((times[i]-avg)/dev)>1.96))
      inliers.push_back(times[i]);
  }
  
  return inliers;
}

void resetDefAndpath(double def[], double path[]){
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

int main(int argc, char **argv){
  ros::init(argc, argv, "pr2_timer_node");
  ros::Subscriber sub;
  
  Arm rarm("right", sub);
  vector<vector<double> > paths;
  vector<double> times;
  double time =0.0, step=1.0;
  double primitive=0.13962634016;
  for(int j = 0; j < 10; j++){
    for(int i = 0; i <3; i++){
      vector<double> v;
      times.push_back(time);
      time+=step;
      for(int j = 0; j < 7; j++){
	v.push_back(0.0);
      }
      paths.push_back(v);
      v.clear();
    }
    /*
      paths[0][1]=primitive;
      paths[1][1]=primitive;
      paths[2][0]=primitive;
      paths[3][2]=primitive;
      paths[4][2]=primitive;
      paths[5][2]=primitive;
      paths[6][3]=-1*primitive;
      paths[7][3]=-1*primitive;
      paths[8][2]=-1*primitive;
      paths[9][1]=-1*primitive;
      paths[10][2]=primitive;
    */
    paths[0][0]=0.0;
    paths[1][0]=1.0;
    paths[2][0]=0.0;
    /*paths[3][0]=1.0;
    paths[4][0]=0.0;
    paths[5][0]=1.0;
    paths[6][0]=0.0;
    paths[7][0]=1.0;
    paths[8][0]=0.0;
    paths[9][0]=1.0;
    paths[10][0]=1.0;*/
    cout<<"The path took "<<rarm.timePath(paths, times)<<endl;
    paths.clear();
    times.clear();
    step-=0.1;
    time=0.0;
    cout<<"Step is "<<step<<endl;
  }
  
  
  /*
  vector<double> joint_time, joint_inliers;
  vector<vector<double> > times, originals;
  double delta0, delta1;
  for(int i = 0; i < 7;i++){
    for(int j = 0; j < 3; j++){
      for(delta0=rarm.min[i], delta1=delta0+primitive;delta1<rarm.max[i];delta0=delta1, delta1+=primitive){
	def[i]=delta0; path[i]=delta1;
	joint_time.push_back(rarm.time(def, path, 0));
      }
    }
    joint_inliers = removeOutliers(joint_time);
    times.push_back(joint_inliers);
    originals.push_back(joint_time);
    joint_inliers.clear();
    joint_time.clear();
    resetDefAndpath(def,path);
  }
  cout<<"Total number of statistical outliers per trial: "<<endl;
  int num_inliers;
  for(int i =0; i < 7; i++){
    num_inliers=originals[i].size()-times[i].size();
    cout<<"Joint "<<i<<": ("<<num_inliers<<" out of "<<originals[i].size()<<") ";
    num_inliers=0;
  }
  
  cout<<endl<<"Average times:"<<endl;
  for(int i = 0; i < 7;i++){
    cout<<average(times[i])<<" ";
  }
  
  cout<<endl;
  */
  /*for(delta0=.564, delta1=.564-primitive;delta1>-2.135;delta0=delta1, delta1-=primitive){
    def[0]=delta0; path[0]=delta1;
    times.push_back(rarm.time(def, path, 0));
    }*//*
  double sum=0.0;
  times[0]=100.0;
  vector<double> inliers = removeOutliers(times);
  for(int i = 0; i < times.size();i++){
    cout<<times[i]<<" ";
  }
  cout<<endl<<endl;
  for(int i = 0; i < inliers.size();i++){
    cout<<inliers[i]<<" ";
  }
  cout<<endl<<endl;
  cout<<"Times: "<<times.size()<<" Inliers: "<<inliers.size()<<endl;
  cout<<endl<<"Average: "<<average(inliers)<<endl;*/
  return 0;
}
