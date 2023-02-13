#include <median_filter/stereo_median_filter.h>
// #include <pcl/filters/median_filter.h>

namespace stereo_pointcloud_preprocessor {

MedianFilter::MedianFilter() {}

MedianFilter::~MedianFilter() {}

void MedianFilter::onInit() {
  NODELET_INFO("MedianFilter Init");
  nh_ = getNodeHandle();
  pnh_ = getPrivateNodeHandle();

  // Publisher
  point_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("median_filter/output", 10);

  // Subscriber
  point_sub_ = nh_.subscribe("input", 1, &MedianFilter::MF_callback, this);

  // param
  pnh_.param<int>("WINDOW_SIZE", window_size_, 3);
  pnh_.param<int>("MAX_MOVE_DIS", max_move_dis_, 100);
}

/**********************************************************************************************************************************************
/関数名：MF_callback1
/low_pass_filter/PointCloud_LPF1がsubscribeされたときのコールバック関数
/入力のpoincloudに対してノイズを除去するmedian_filterを行う
/処理後の点群をmedian_filter/PointCloud_MF1としてPublishする
/入力：const sensor_msgs::PointCloud2ConstPtr& point1
/出力：void
***********************************************************************************************************************************************/
void MedianFilter::MF_callback(const sensor_msgs::PointCloud2ConstPtr &point1) {
  //入力をpclで扱える形に変換
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_in(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*point1, *cloud1_in);

  // pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud1_t = point1;
  // cloud1_t = point1;
  // NODELET_WARN("%f", point1->header.stamp.sec);
  //点群のポイントを宣言
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1_t(
  //     new pcl::PointCloud<pcl::PointXYZRGB>(cloud1_));
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_out(
      new pcl::PointCloud<pcl::PointXYZ>(cloud1_));

  mf_.setInputCloud(cloud1_in);

  //フィルターを行うサイズを指定
  mf_.setWindowSize(window_size_);
  //点の移動距離の最大値を指定
  mf_.setMaxAllowedMovement(max_move_dis_);
  //フィルターを行う
  mf_.filter(*cloud1_out);
  // publish
  point_pub_.publish(cloud1_);
}

} // namespace stereo_pointcloud_preprocessor

PLUGINLIB_EXPORT_CLASS(stereo_pointcloud_preprocessor::MedianFilter,
                       nodelet::Nodelet);
