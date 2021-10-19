#include <sac_segmentation_filter/sac_segmentation_filter.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "sac_segmentation_filter_node");
  sac_segmentation_filter_node::Sac_Segmentation_Filter sag_seg;
  ros::spin();
}