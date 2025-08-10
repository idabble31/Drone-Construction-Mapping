#include <ros/ros.h>
#include "drone_middleware/pcl_merge.hpp"  // Your header file

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcl_merge");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");  // private namespace for params

    PCLMergeNode merger(nh, pnh);

    ros::spin();
    return 0;
}
