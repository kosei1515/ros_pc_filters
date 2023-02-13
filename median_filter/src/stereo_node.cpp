#include <median_filter/stereo_median_filter.h>

int main(int argc, char **argv) {
  //初期化＆ノード名をimage_rotateに設定
  ros::init(argc, argv, "stereo_median_filter_node");
  //オブジェクトを宣言
  stereo_pointcloud_preprocessor::MedianFilter SMF;
  // Ctrl+Cが押されるまたはros::shutdown()が起こらない限りloop
  ros::spin();
  return 0;
}