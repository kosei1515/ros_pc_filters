#pragma once
#include <pcl/filters/median_filter.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace stereo_pointcloud_preprocessor {
class MedianFilter : public nodelet::Nodelet {
private:
  ros::Publisher point_pub_;
  ros::Subscriber point_sub_;
  ros::NodeHandle nh_, pnh_;
  pcl::PointCloud<pcl::PointXYZ> cloud1_;
  pcl::MedianFilter<pcl::PointXYZ> mf_;
  //データがないことを表すnanを代入
  const float invalid = std::numeric_limits<float>::quiet_NaN();

  // parameter
  int window_size_;
  int max_move_dis_;

public:
  MedianFilter();
  ~MedianFilter();
  virtual void onInit();
  void MF_callback(const sensor_msgs::PointCloud2ConstPtr &point1);
};

} // namespace stereo_pointcloud_preprocessor