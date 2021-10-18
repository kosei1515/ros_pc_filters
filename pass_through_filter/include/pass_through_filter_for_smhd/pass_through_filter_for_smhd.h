#include <pcl_ros/point_cloud.h>
#include<sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>  
#include <visualization_msgs/MarkerArray.h>

namespace pass_through_filter_for_smhd_node
{
    class Pass_Through_Filter_For_Smhd
    {
    private:
        ros::NodeHandle nh_, pnh_;

        ros::Subscriber pc_sub_;
        ros::Publisher pc_pub_;

        //parameters
        float x_min_;
        float x_max_;
        float y_min_;
        float y_max_;


    public:
        Pass_Through_Filter_For_Smhd();
        ~Pass_Through_Filter_For_Smhd();

        void pc_callback(const sensor_msgs::PointCloud2ConstPtr &pcl_msg);
    };
    
} // namespace pass_through_filter_node
