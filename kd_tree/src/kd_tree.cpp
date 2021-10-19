#include <kd_tree/kd_tree.h>

#define node_count 8
#define voxel_size 0.03

namespace kd_tree_node {
KD_Tree::KD_Tree() {
  // subscribe xy lattice point
  sub_planning_pc_ = nh_.subscribe("your_lattice_pc_topic",
                                   10, &KD_Tree::planning_pc_callback, this);
  // you should subscribe your point cloud topic here
  sub_depth_pc_ = nh_.subscribe("your_pc_topic", 10,
                                &KD_Tree::depth_pc_callback, this);
}
KD_Tree::~KD_Tree() {}

void KD_Tree::planning_pc_callback(
    const sensor_msgs::PointCloud2ConstPtr &p_pc_msg) {

  // get XYZRGB object
  pcl::fromROSMsg<pcl::PointXYZRGB>(*p_pc_msg, p_pc_in_);
  pcl::fromROSMsg<pcl::PointXYZRGB>(*p_pc_msg, p_pc_in2_);
  // for kd tree, get xy object
  pcl::fromROSMsg(*p_pc_msg, p_pc_xy_in_);
  pcl::fromROSMsg(*p_pc_msg, p_pc_xy_in2_);
}

void KD_Tree::depth_pc_callback(
    const sensor_msgs::PointCloud2ConstPtr &d_pc_msg) {

  // PC pointer
  pcl::PointCloud<pcl::PointXYZ>::Ptr d_pc_ptr(
      new pcl::PointCloud<pcl::PointXYZ>(d_pc_in_));
  pcl::PointCloud<pcl::PointXY>::Ptr d_pc_xy_ptr(
      new pcl::PointCloud<pcl::PointXY>(d_pc_xy_in_));

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pc_ptr(
      new pcl::PointCloud<pcl::PointXYZRGB>(p_pc_in_));

  // Planning pc pointer
  pcl::PointCloud<pcl::PointXY>::Ptr p_pc_xy_ptr(
      new pcl::PointCloud<pcl::PointXY>(p_pc_xy_in_));

  // KD tree object
  pcl::search::KdTree<pcl::PointXY>::Ptr kdtree_ptr(
      new pcl::search::KdTree<pcl::PointXY>);

  // This vector will store the output neighbors.
  std::vector<int> pointIndices;
  // This vector will store their squared distances to the search point.
  std::vector<float> squaredDistances;

  // get XYZ object
  pcl::fromROSMsg(*d_pc_msg, *d_pc_ptr);
  // get XY object for kd tree
  pcl::fromROSMsg(*d_pc_msg, *d_pc_xy_ptr);

  // set xy pc in kd tree ptr
  kdtree_ptr->setInputCloud(d_pc_xy_ptr);

  // Search neighbors at every planning pcs
  for (int i = 0; i < p_pc_ptr->points.size(); i++) {
    // Search (node_count) nodes in (voxel_size) radius from points
    if (kdtree_ptr->radiusSearch(p_pc_xy_ptr->points[i], voxel_size,
                                 pointIndices, squaredDistances,
                                 node_count) == node_count) {
      // If (node count) nodes are detected, set height(z) in planning
      // pc(xyzrgb)
      float sum = 0;
      for (size_t j = 0; j < pointIndices.size(); j++) {
        sum += d_pc_ptr->points[pointIndices[j]].z;
      }
      // get average z of all detected pc in voxel_size
      p_pc_ptr->points[i].z = sum / node_count;
      p_pc_ptr->points[i].r = 0;
      p_pc_ptr->points[i].g = 0;
      p_pc_ptr->points[i].b = 200;
    } else {
      p_pc_ptr->points[i].z = NAN;
    }
  }
  // ROS_INFO("kd_tree filtered");
  pub_pc_.publish(*p_pc_ptr);
}


} // namespace kd_tree_node
