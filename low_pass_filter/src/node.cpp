#include <low_pass_filter/low_pass_filter.h>

int main(int argc, char **argv) {
  //初期化＆ノード名をimage_rotateに設定
  ros::init(argc, argv, "low_pass_filter_node");
  //オブジェクトを宣言
  pointcloud_preprocessor::LowPassFilter LP;
  // Ctrl+Cが押されるまたはros::shutdown()が起こらない限りloop
  ros::spin();
  return 0;
}