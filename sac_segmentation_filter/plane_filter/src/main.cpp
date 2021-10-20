#include <plane_filter/plane_filter.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "plane_filter_node");
  plane_filter_node::Plane_Filter sag_seg;
  ros::spin();
}