#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <iostream>
#include <dirent.h>
#include <string>
#include <vector>
#include <algorithm>
#include <fstream>

using namespace std;
using namespace pcl;
using namespace ros;


int read_lidar_single_thread(int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "read_lidar");
  ros::NodeHandle nh("~");

  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2> ("lidar", 5);
  Rate rate(10);
  // if(argc < 2){
  //   cerr << "no dir\n";
  //   return -1;
  // }
  // // argv[1]: directory
  // cout << "dir: " << argv[1] << endl;
  // DIR *dir = opendir(argv[1]);
  string dir_s;
  nh.getParam("/Lidar/directory", dir_s);
  DIR *dir = opendir(dir_s.c_str());
  dirent *file;
  vector<string> files;
  // read names of all files
  while(file = readdir(dir)){
    if(file->d_name[0] == '.'){
      continue;
    }
    files.push_back(dir_s + "/" + string(file->d_name));
  }
  closedir(dir);
  // sort names
  sort(files.begin(), files.end());
  cout << "first file: " << files[0] << endl;
  cout << "file number: " << files.size() << endl;

  sensor_msgs::PointCloud2 cloud_ros;
  cloud_ros.header.frame_id = "velodyne";
  cloud_ros.header.stamp = ros::Time::now();
  ros::Duration stamp_duration(0.1);

  // for each file
  for(vector<string>::iterator iter = files.begin(); iter != files.end(); ++iter){
    // load point cloud
    fstream input(iter->c_str(), ios::in | ios::binary);
    if(!input.good()){
      cerr << "Could not read file: " << *iter << endl;
      return -2;
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
    pcl_conversions::fromPCL(*cloud2, cloud_ros);
    cloud_ros.header.stamp+=stamp_duration;
    pub.publish(cloud_ros);
    static int i=0;
    i++;
    if(i%100==0)cout<<i<<endl;
    spinOnce();
    rate.sleep();
  }

  return 0;
}
