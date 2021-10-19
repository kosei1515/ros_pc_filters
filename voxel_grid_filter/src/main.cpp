#include <ros/ros.h>
#include <voxel_grid_filter/voxel_grid_filter.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "voxel_grid_filter_node");
  voxel_grid_filter_node::Voxel_Grid_Filter vg_filter;

  ros::spin();
}