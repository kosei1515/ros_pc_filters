#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace plane_filter_node {
class Plane_Filter {
private:
  ros::NodeHandle nh_, pnh_;
  ros::Publisher pc_pub_;
  ros::Subscriber pc_sub_;

  // parameters
  float distance_threshould_;
  bool is_get_plane_;

  // object
  pcl::PointCloud<pcl::PointXYZ> pc_in_;
  pcl::PointCloud<pcl::PointXYZ> pc_out_;

public:
  Plane_Filter();
  ~Plane_Filter();

  void pc_callback(const sensor_msgs::PointCloud2ConstPtr &pc_msg);
};

} // namespace plane_filter_node
