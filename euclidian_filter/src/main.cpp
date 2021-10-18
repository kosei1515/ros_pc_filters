#include <euclidean_cluster/euclidean_cluster.h>

int main(int argc, char** argv){
    ros::init(argc,argv,"euclidean_cluster_node");
    euclidean_cluster_node::Euclidean_Cluster ecvpep;
    ROS_INFO("start");
    ros::spin();
}