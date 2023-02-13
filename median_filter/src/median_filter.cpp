#include <median_filter/median_filter.h>
// #include <pcl/filters/median_filter.h>

namespace pointcloud_preprocessor {

MedianFilter::MedianFilter() {}
MedianFilter::~MedianFilter() {}

void MedianFilter::onInit() {
  NODELET_INFO("MedianFilter Init");
  nh_ = getNodeHandle();
  pnh_ = getPrivateNodeHandle();

  // Publisher
  point_pub1_ = nh_.advertise<sensor_msgs::PointCloud2>(
      "median_filter/PointCloud_MF1", 10);
  point_pub2_ = nh_.advertise<sensor_msgs::PointCloud2>(
      "median_filter/PointCloud_MF2", 10);
  point_pub3_ = nh_.advertise<sensor_msgs::PointCloud2>(
      "median_filter/PointCloud_MF3", 10);

  // Subscriber
  point_sub1_ = nh_.subscribe("low_pass_filter/PointCloud_LPF1", 10,
                              &MedianFilter::MF_callback1, this);
  point_sub2_ = nh_.subscribe("low_pass_filter/PointCloud_LPF2", 10,
                              &MedianFilter::MF_callback2, this);
  point_sub3_ = nh_.subscribe("low_pass_filter/PointCloud_LPF3", 10,
                              &MedianFilter::MF_callback3, this);

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
void MedianFilter::MF_callback1(
    const sensor_msgs::PointCloud2ConstPtr &point1) {
  //入力をpclで扱える形に変換
  pcl::fromROSMsg(*point1, cloud1_);
  // NODELET_WARN("%f", point1->header.stamp.sec);
  //点群のポイントを宣言
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud1_t(
      new pcl::PointCloud<pcl::PointXYZ>(cloud1_));
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_out(
      new pcl::PointCloud<pcl::PointXYZ>(cloud1_));
  pcl::MedianFilter<pcl::PointXYZ> mf;

  mf.setInputCloud(cloud1_t);

  //フィルターを行うサイズを指定
  mf.setWindowSize(window_size_);
  //点の移動距離の最大値を指定
  mf.setMaxAllowedMovement(max_move_dis_);
  //フィルターを行う
  mf.filter(*cloud1_out);
  // publish
  point_pub1_.publish(cloud1_);
}

/**********************************************************************************************************************************************
/関数名：MF_callback2
/low_pass_filter/PointCloud_LPF2がsubscribeされたときのコールバック関数
/入力のpoincloudに対してノイズを除去するmedian_filterを行う
/処理後の点群をmedian_filter/PointCloud_MF2としてPublishする
/入力：const sensor_msgs::PointCloud2ConstPtr& point1
/出力：void
***********************************************************************************************************************************************/

void MedianFilter::MF_callback2(
    const sensor_msgs::PointCloud2ConstPtr &point1) {
  //入力をpclで扱える形に変換
  pcl::fromROSMsg(*point1, cloud1_);
  //点群のポイントを宣言
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud1_t(
      new const pcl::PointCloud<pcl::PointXYZ>(cloud1_));
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_out(
      new pcl::PointCloud<pcl::PointXYZ>(cloud1_));

  pcl::MedianFilter<pcl::PointXYZ> mf;

  //   ROS_INFO("start");

  mf.setInputCloud(cloud1_t);

  //フィルターを行うサイズを指定
  mf.setWindowSize(window_size_);
  //点の移動距離の最大値を指定
  mf.setMaxAllowedMovement(max_move_dis_);
  //フィルターを行う
  mf.filter(*cloud1_out);
  // publish
  point_pub2_.publish(cloud1_);
}

/**********************************************************************************************************************************************
/関数名：MF_callback2
/low_pass_filter/PointCloud_LPF2がsubscribeされたときのコールバック関数
/入力のpoincloudに対してノイズを除去するmedian_filterを行う
/処理後の点群をmedian_filter/PointCloud_MF2としてPublishする
/入力：const sensor_msgs::PointCloud2ConstPtr& point1
/出力：void
***********************************************************************************************************************************************/

void MedianFilter::MF_callback3(
    const sensor_msgs::PointCloud2ConstPtr &point1) {
  //入力をpclで扱える形に変換
  pcl::fromROSMsg(*point1, cloud1_);
  //点群のポイントを宣言
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud1_t(
      new pcl::PointCloud<pcl::PointXYZ>(cloud1_));
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_out(
      new pcl::PointCloud<pcl::PointXYZ>(cloud1_));

  pcl::MedianFilter<pcl::PointXYZ> mf;

  mf.setInputCloud(cloud1_t);

  //フィルターを行うサイズを指定
  mf.setWindowSize(window_size_);
  //点の移動距離の最大値を指定
  mf.setMaxAllowedMovement(max_move_dis_);
  //フィルターを行う
  mf.filter(*cloud1_out);
  // publish
  point_pub3_.publish(cloud1_);
}
} // namespace pointcloud_preprocessor

PLUGINLIB_EXPORT_CLASS(pointcloud_preprocessor::MedianFilter, nodelet::Nodelet);
