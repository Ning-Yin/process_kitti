#include "read_lidar.h"
#include <dirent.h>
#include <algorithm>
#include <fstream>
#include <iostream>

using namespace std;
using namespace pcl;
using namespace ros;

Bin2Rosbag::Bin2Rosbag(NodeHandle &nh){
  publisher=nh.advertise<sensor_msgs::PointCloud2>(publish_topic, 1);
  string dir_s;
  nh.getParam(param_name, dir_s);
  DIR *dir=opendir(dir_s.c_str());
  dirent *file;
  while(file=readdir(dir)){
    if(file->d_name[0]=='.'){
      continue;
    }
    files.push_back(dir_s+"/"+string(file->d_name));
  }
  closedir(dir);
  sort(files.begin(),files.end());
  convert_thread.reset(new thread(&Bin2Rosbag::readBinFilesAndConvert,this));
  publish_thread.reset(new thread(&Bin2Rosbag::publishRosCloud,this));
  convert_thread->join();
  publish_thread->join();
}

void Bin2Rosbag::readBinFilesAndConvert(){
  Time time=Time::now();
  Duration stamp_duration(0.1);
  for(vector<string>::iterator iter = files.begin(); iter != files.end(); ++iter){
    // load point cloud
    fstream input(iter->c_str(), ios::in | ios::binary);
    if(!input.good()){
      cerr << "Could not read file: " << *iter << endl;
      notEnd=false;
      return;
    }
    input.seekg(0, ios::beg);

    PointCloud<PointXYZI>::Ptr cloud(new PointCloud<PointXYZI>);

    for (; input.good() && !input.eof();) {
      PointXYZI point;
      input.read((char *) &point.x, 3*sizeof(float));
      input.read((char *) &point.intensity, sizeof(float));
      cloud->push_back(point);
    }
    PCLPointCloud2::Ptr cloud2(new PCLPointCloud2);
    toPCLPointCloud2(*cloud,*cloud2);
    unique_lock<mutex> lock(m);
    cv.wait(lock,[this]()->bool{return canReadFile;});
    pcl_conversions::fromPCL(*cloud2, cloud_ros);
    cloud_ros.header.frame_id = header_frame_id;
    cloud_ros.header.stamp = time;
    canReadFile=false;
    cv.notify_one();
    time+=stamp_duration;
  }
  notEnd=false;
}

void Bin2Rosbag::publishRosCloud(){
  std::chrono::steady_clock::time_point last_time=chrono::steady_clock::now(), this_time;
  int i=0;
  while(notEnd){
    publish_rate.sleep();
    {
      unique_lock<mutex> lock(m);
      cv.wait(lock,[this]()->bool{return !canReadFile;});
      publisher.publish(cloud_ros);
      canReadFile=true;
    }
    spinOnce();
    cv.notify_one();
    this_time=chrono::steady_clock::now();
    cout<<++i<<"\t"<<chrono::duration_cast<chrono::milliseconds>(this_time-last_time).count()<<"ms\n";
    last_time=this_time;
  }
}
