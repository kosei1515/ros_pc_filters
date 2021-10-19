#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace voxel_grid_filter_node {
class Voxel_Grid_Filter {
private:
  ros::NodeHandle nh_, pnh_;

  ros::Subscriber pc_sub_;
  ros::Publisher vg_pub_;

  // parameters
  float leaf_size_x_;
  float leaf_size_y_;
  float leaf_size_z_;

  // objects
  pcl::PointCloud<pcl::PointXYZ> pc_in_;
  pcl::PointCloud<pcl::PointXYZ> pc_out_;

public:
  Voxel_Grid_Filter();
  ~Voxel_Grid_Filter();

  void pc_callback(const sensor_msgs::PointCloud2ConstPtr &pc_msg);
};

} // namespace voxel_grid_filter_node
