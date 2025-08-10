#pragma once
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class PCLMergeNode
{
public:
    PCLMergeNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:
    void pointCloudCallback(
        const sensor_msgs::PointCloud2ConstPtr& cloud1,
        const sensor_msgs::PointCloud2ConstPtr& cloud2);

    ros::NodeHandle nh_, pnh_;
    std::vector<std::string> lidar_topics_;
    ros::Publisher pub_;

    message_filters::Subscriber<sensor_msgs::PointCloud2> sub1_, sub2_;
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::PointCloud2,
        sensor_msgs::PointCloud2
    > MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync_;
};
