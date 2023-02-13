#include <median_filter/median_filter.h>

int main(int argc, char **argv) {
  //初期化＆ノード名をimage_rotateに設定
  ros::init(argc, argv, "median_filter_node");
  //オブジェクトを宣言
  pointcloud_preprocessor::MedianFilter MF;
  // Ctrl+Cが押されるまたはros::shutdown()が起こらない限りloop
  ros::spin();
  return 0;
}