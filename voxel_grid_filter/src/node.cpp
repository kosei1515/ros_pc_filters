#include <ros/ros.h>
#include <voxel_grid_filter/voxel_grid_filter.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "voxel_grid_filter_node");
  pointcloud_preprocessor::VoxelGridFilter vg_filter;

  ros::spin();
}