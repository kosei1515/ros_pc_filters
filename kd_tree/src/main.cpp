#include <kd_tree/kd_tree.h>
#include <ros/ros.h>

// Main Loop
int main(int argc, char **argv) {
  ros::init(argc, argv, "kd_tree_node");
  // ROS_INFO("start");
  kd_tree_node::KD_Tree kd_tree;
  ros::spin();
  return 0;
}