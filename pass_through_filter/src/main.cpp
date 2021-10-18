#include<ros/ros.h>
#include <pass_through_filter_for_smhd/pass_through_filter_for_smhd.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "pass_through_filter_for_smhd_node");
    pass_through_filter_for_smhd_node::Pass_Through_Filter_For_Smhd pass_through;

    ros::spin();
    
}