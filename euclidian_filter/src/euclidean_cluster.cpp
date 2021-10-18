#include <euclidean_cluster/euclidean_cluster.h>

namespace euclidean_cluster_node
{

Euclidean_Cluster::Euclidean_Cluster(): nh_("~"),pnh_("~")
{
  
  pnh_.param<std::string>("target_frame", target_frame_, "base_link");
  pnh_.param<bool>("use_height", use_height_, false);
  pnh_.param<int>("min_cluster_size", min_cluster_size_, 3);
  pnh_.param<int>("max_cluster_size", max_cluster_size_, 200);
  pnh_.param<float>("tolerance", tolerance_, 1.0);

  pnh_.param<float>("min_height", min_height_, 0.5);
  pnh_.param<float>("max_height", max_height_, 2.0);
  pnh_.param<float>("limit_height",limit_height_, 0.1);
  pnh_.param<float>("limit_width", limit_width_, 1.5);
  pnh_.param<float>("limit_length", limit_length_, 1.5);
  pnh_.param<float>("front_lidar_height", front_lidar_height_, 0.7);
  pnh_.param<float>("max_detection_distance", max_detection_distance_, 5.0);
  

  pointcloud_sub_ =
    nh_.subscribe("input", 10, &Euclidean_Cluster::pointcloudCallback, this);

  cluster_pub_ =
    nh_.advertise<sensor_msgs::PointCloud2>("output", 10);
  debug_pub_ = pnh_.advertise<sensor_msgs::PointCloud2>("euclidean_cluster_ver_pedestrian_extract_project_node/clusters", 1);
}

Euclidean_Cluster::~Euclidean_Cluster()
{
}

void Euclidean_Cluster::pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr &input_msg)
{
  // convert ros to pcl
  pcl::PointCloud<pcl::PointXYZ>::Ptr raw_pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input_msg, *raw_pointcloud_ptr);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  if (!use_height_) {
    for (size_t i = 0; i < raw_pointcloud_ptr->size(); ++i) {
      pcl::PointXYZ point;
      point.x = raw_pointcloud_ptr->points.at(i).x;
      point.y = raw_pointcloud_ptr->points.at(i).y;
      point.z = 0.0;
      pointcloud_ptr->push_back(point);
    }
  } else {
    pointcloud_ptr = raw_pointcloud_ptr;
  }

  // create tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(pointcloud_ptr);

  // clustering
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> pcl_euclidean_cluster;
  pcl_euclidean_cluster.setClusterTolerance(tolerance_);
  pcl_euclidean_cluster.setMinClusterSize(min_cluster_size_);
  pcl_euclidean_cluster.setMaxClusterSize(max_cluster_size_);
  pcl_euclidean_cluster.setSearchMethod(tree);
  pcl_euclidean_cluster.setInputCloud(pointcloud_ptr);
  pcl_euclidean_cluster.extract(cluster_indices);

  
    //debug_pub_.publish(pointcloud_output);
    ROS_INFO("pub");
  }

} // namespace euclidean_cluster_ver_pedestrian_extract_project_node

