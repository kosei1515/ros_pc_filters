#pragma once
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

using namespace std;

namespace pointcloud_preprocessor {
class LowPassFilter : public nodelet::Nodelet {
private:
  ros::NodeHandle nh_, pnh_;
  ros::Publisher point_pub_;
  ros::Subscriber point_sub_;

  // parameter
  bool flag_;
  float dis_lim_;
  float filter_rate_;

  const int num_ = 114912;
  float last_data1_[114912], last_data2_[114912], last_data3_[114912];

public:
  LowPassFilter();
  ~LowPassFilter();
  virtual void onInit();
  void LPF_callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pc_msg);
};

} // namespace pointcloud_preprocessor
