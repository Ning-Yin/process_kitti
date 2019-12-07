#pragma once
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <vector>
#include <string>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <memory>

class Bin2Rosbag{
public:
  Bin2Rosbag(ros::NodeHandle &nh);

protected:
  void readBinFilesAndConvert();
  void publishRosCloud();

private:
  volatile bool canReadFile=true;
  volatile bool notEnd=true;
  std::mutex m;
  std::condition_variable cv;
  ros::Publisher publisher;
  std::vector<std::string> files;
  std::unique_ptr<std::thread> convert_thread;
  std::unique_ptr<std::thread> publish_thread;
  sensor_msgs::PointCloud2 cloud_ros;

  std::string param_name="/Lidar/directory";
  std::string header_frame_id="velodyne";
  std::string publish_topic="velodyne_points";
  ros::Rate publish_rate=10;//10 Hz
  ros::Duration stamp_duration=ros::Duration(0.1);
};
