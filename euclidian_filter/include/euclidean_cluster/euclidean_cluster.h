#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>

namespace euclidean_cluster_node
{
class Euclidean_Cluster
{
private:
ros::NodeHandle nh_, pnh_;
  ros::Subscriber pointcloud_sub_;
    
  ros::Publisher cluster_pub_;
  ros::Publisher debug_pub_;

  std::string target_frame_;
  bool use_height_;
  int min_cluster_size_;
  int max_cluster_size_;
  float tolerance_;

  float min_height_;
  float max_height_;
  float limit_height_;
  float limit_width_;
  float limit_length_;
  float front_lidar_height_;
  float max_detection_distance_;

  //void make_cluster_list(pcl::PointCloud<pcl::PointXYZ> &points, std::vector<pcl::PointIndices> &indices);

  
public:
    Euclidean_Cluster();
    ~Euclidean_Cluster();

    void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr & input_msg);
};



} // namespace euclidean_cluster_node
