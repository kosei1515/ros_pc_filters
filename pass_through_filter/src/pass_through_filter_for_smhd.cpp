#include <pass_through_filter_for_smhd/pass_through_filter_for_smhd.h>

namespace pass_through_filter_for_smhd_node
{
    Pass_Through_Filter_For_Smhd::Pass_Through_Filter_For_Smhd(): nh_("~"), pnh_("~")
    {
        pc_sub_=nh_.subscribe("input", 10, &Pass_Through_Filter_For_Smhd::pc_callback, this);
        pc_pub_=nh_.advertise<sensor_msgs::PointCloud2>("output", 10);

        pnh_.param<float>("x_min", x_min_, -5.0);
        pnh_.param<float>("x_max", x_max_, 5.0);
        pnh_.param<float>("y_min", y_min_, -5.0);
        pnh_.param<float>("y_max", y_max_, 5.0);
        // pnh_.param<float>("z_min", z_min_, -5.0);
        // pnh_.param<float>("z_max", z_max_, 5.0);
        
    }
    
    Pass_Through_Filter_For_Smhd::~Pass_Through_Filter_For_Smhd()
    {
    }

    void Pass_Through_Filter_For_Smhd::pc_callback(const sensor_msgs::PointCloud2ConstPtr &pc_msg){
        pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ptr(new(pcl::PointCloud<pcl::PointXYZ>));
        pcl::PointCloud<pcl::PointXYZ>::Ptr pc_out_ptr(new(pcl::PointCloud<pcl::PointXYZ>));
        pcl::PointCloud<pcl::PointXYZ>::Ptr pc_out2_ptr(new(pcl::PointCloud<pcl::PointXYZ>));
        pcl::PassThrough<pcl::PointXYZ> pass_filter_x, pass_filter_y;

        pcl::fromROSMsg(*pc_msg, *pc_ptr);
        
        pass_filter_x.setFilterFieldName("x");  // x軸（奥行き）の値でフィルタをかける
        pass_filter_x.setFilterLimits(x_min_, x_max_);
        pass_filter_y.setFilterFieldName("y");
        pass_filter_y.setFilterLimits(y_min_,x_min_);

        pass_filter_x.setInputCloud(pc_ptr);
        pass_filter_x.filter(*pc_out_ptr);

        pass_filter_y.setInputCloud(pc_out_ptr);
        pass_filter_y.filter(*pc_out2_ptr);

        pc_pub_.publish(pc_out2_ptr);
    }
} // namespace pass_through_filter_for_smhd_node

