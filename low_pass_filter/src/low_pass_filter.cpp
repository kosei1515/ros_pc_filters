#include <low_pass_filter/low_pass_filter.h>

using namespace std;

namespace pointcloud_preprocessor {
LowPassFilter::LowPassFilter() : nh_(), pnh_() {}
LowPassFilter::~LowPassFilter() {}

void LowPassFilter::onInit() {
  NODELET_INFO("LowPassFilter Init");
  nh_ = getNodeHandle();
  pnh_ = getPrivateNodeHandle();

  // Publisher
  point_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("low_pass_filter/output", 10);

  // Subscriber
  point_sub_ = nh_.subscribe("input", 1, &LowPassFilter::LPF_callback, this);

  // param
  pnh_.param<bool>("FLAG1", flag_, false);
  pnh_.param<float>("DIS_LIM", dis_lim_, 4);
  pnh_.param<float>("FILTER_RATE", filter_rate_, 0.5);
}

/**********************************************************************************************************************************************
/関数名：LPF_callback1
/pico_flexx/pointsがsubscribeされたときのコールバック関数
/入力のpoincloudにたいして高周波のノイズを除去するLowPassFilterを行う
/処理後の点群をLowPassFilter/PointCloud_LPF1としてPublishする
/入力：const sensor_msgs::PointCloud2ConstPtr& point1
/出力：void
***********************************************************************************************************************************************/

void LowPassFilter::LPF_callback(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pc_msg) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_data(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  *cloud_data = *pc_msg;
  // pcl::fromROSMsg(*point1, cloud_data);
  //   NODELET_WARN("%f", cloud_data.header.stamp);

  //   std::cout << point1.header.stamp << std::endl;
  //   std::cout << cloud_data.header.stamp << std::endl;
  //   NODELET_WARN("%f", point1.header.stamp);
  //   NODELET_WARN("%f", cloud_data.header.stamp);

  //初回のみ、前回データがないため今回取得したデータを前回データとしても使用するため代入
  if (flag_ == false) {
    for (int i = 0; i < num_ / 3; i++) {
      last_data1_[i * 3] = cloud_data->points[i].x;
      last_data1_[i * 3 + 1] = cloud_data->points[i].y;
      last_data1_[i * 3 + 2] = cloud_data->points[i].z;
    }
    flag_ = true;
  }

  //ローパスフィルタの処理開始
  for (int i = 0; i < num_ / 3; i++) {
    if ((!(std::isnan(cloud_data->points[i].x))) &&
        (!(std::isnan(last_data1_[i * 3])))) {
      float x_dis = (cloud_data->points[i].x - last_data1_[i * 3]);
      float y_dis = (cloud_data->points[i].y - last_data1_[i * 3 + 1]);
      float z_dis = (cloud_data->points[i].z - last_data1_[i * 3 + 2]);
      //前回データとの距離計算
      float dis = sqrt(x_dis * x_dis + y_dis * y_dis + z_dis * z_dis);

      if (dis > dis_lim_) {
        //前回データに値を格納
        last_data1_[i * 3] = cloud_data->points[i].x;
        last_data1_[i * 3 + 1] = cloud_data->points[i].y;
        last_data1_[i * 3 + 2] = cloud_data->points[i].z;
        // nanにする
        cloud_data->points[i].x = NAN;
        cloud_data->points[i].y = NAN;
        cloud_data->points[i].z = NAN;
        //データがnanであるというカウントを+1
      } else {
        // LowPassFilterをかける
        cloud_data->points[i].x = filter_rate_ * last_data1_[i * 3] +
                                  (1 - filter_rate_) * cloud_data->points[i].x;
        cloud_data->points[i].y = filter_rate_ * last_data1_[i * 3 + 1] +
                                  (1 - filter_rate_) * cloud_data->points[i].y;
        cloud_data->points[i].z = filter_rate_ * last_data1_[i * 3 + 2] +
                                  (1 - filter_rate_) * cloud_data->points[i].z;
      }
    }
    //前回データを更新
    last_data1_[i * 3] = cloud_data->points[i].x;
    last_data1_[i * 3 + 1] = cloud_data->points[i].y;
    last_data1_[i * 3 + 2] = cloud_data->points[i].z;
  }
  // pcl_conversions::toPCL(point1.header.stamp, cloud_data->header.stamp);
  // NODELET_WARN("%d", cloud_data->header.stamp);
  // publish
  point_pub_.publish(*cloud_data);
}

} // namespace stereo_pointcloud_preprocessor

PLUGINLIB_EXPORT_CLASS(pointcloud_preprocessor::LowPassFilter,
                       nodelet::Nodelet);
