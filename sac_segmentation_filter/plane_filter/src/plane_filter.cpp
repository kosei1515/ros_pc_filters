#include <plane_filter/plane_filter.h>

namespace plane_filter_node {
Plane_Filter::Plane_Filter() : nh_("~"), pnh_("~") {
  pc_sub_ = nh_.subscribe("input", 10, &Plane_Filter::pc_callback, this);
  pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("output", 10);

  pnh_.param<float>("distance_threshould", distance_threshould_, 0.1);
  pnh_.param<bool>("is_get_plane", is_get_plane_, true);
}

Plane_Filter::~Plane_Filter() {}

void Plane_Filter::pc_callback(const sensor_msgs::PointCloud2ConstPtr &pc_msg) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_in_ptr(
      new pcl::PointCloud<pcl::PointXYZ>(pc_in_));
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_out_ptr(
      new pcl::PointCloud<pcl::PointXYZ>(pc_out_));

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr indices(new pcl::PointIndices);

  pcl::ExtractIndices<pcl::PointXYZ> extract_indices;
  pcl::SACSegmentation<pcl::PointXYZ> seg;

  pcl::fromROSMsg(*pc_msg, *pc_in_ptr);
  // optimal
  seg.setOptimizeCoefficients(true);

  // mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(distance_threshould_);

  seg.setInputCloud(pc_in_ptr);
  seg.segment(*indices, *coefficients);

  // filter
  extract_indices.setInputCloud(pc_in_ptr);
  extract_indices.setIndices(indices);
  extract_indices.setNegative(is_get_plane_);
  extract_indices.filter(*pc_out_ptr);

  pc_pub_.publish(*pc_out_ptr);
}
} // namespace plane_filter_node
