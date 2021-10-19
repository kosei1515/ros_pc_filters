#include <voxel_grid_filter/voxel_grid_filter.h>

namespace voxel_grid_filter_node {
Voxel_Grid_Filter::Voxel_Grid_Filter() : nh_("~"), pnh_("~") {
  pc_sub_ = nh_.subscribe("input", 10, &Voxel_Grid_Filter::pc_callback, this);
  vg_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("output", 10);

  pnh_.param<float>("leaf_size_x", leaf_size_x_, 0.1);
  pnh_.param<float>("leaf_size_y", leaf_size_y_, 0.1);
  pnh_.param<float>("leaf_size_z", leaf_size_z_, 0.1);
}

Voxel_Grid_Filter::~Voxel_Grid_Filter() {}

void Voxel_Grid_Filter::pc_callback(
    const sensor_msgs::PointCloud2ConstPtr &pc_msg) {

  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_in_ptr(
      new pcl::PointCloud<pcl::PointXYZ>(pc_in_));
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_out_ptr(
      new pcl::PointCloud<pcl::PointXYZ>(pc_out_));

  // voxel grid filter
  pcl::VoxelGrid<pcl::PointXYZ> vg;

  // pclで扱えるように点群情報を変換
  pcl::fromROSMsg(*pc_msg, *pc_in_ptr);
  // filterをかける点群を設定
  vg.setInputCloud(pc_in_ptr);
  //リーフサイズを設定
  vg.setLeafSize(leaf_size_x_, leaf_size_y_, leaf_size_z_);
  //ダウンサンプリングを行うか設定
  vg.setDownsampleAllData(false);

  //フィルターをかけ引数に出力
  vg.filter(*pc_out_ptr);
}
} // namespace voxel_grid_filter_node
