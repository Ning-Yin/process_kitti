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

#include "read_lidar.h"

using namespace std;
using namespace pcl;
using namespace ros;

// int read_lidar_single_thread(int argc, char** argv);

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "read_lidar");
  ros::NodeHandle nh("~");

  Bin2Rosbag test(nh);

  return 0;

  // return read_lidar_single_thread(argc, argv);
}
