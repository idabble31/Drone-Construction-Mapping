#include "drone_middleware/pcl_merge.hpp"

PCLMergeNode::PCLMergeNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh)
{
    // Read parameter
    if (!pnh_.getParam("lidar_topics", lidar_topics_)) {
        ROS_ERROR("Parameter 'lidar_topics' not found!");
        ros::shutdown();
        return;
    }

    if (lidar_topics_.size() != 2) {
        ROS_ERROR("This version only supports merging exactly 2 LiDAR topics!");
        ros::shutdown();
        return;
    }

    // Subscribers with message_filters
    sub1_.subscribe(nh_, lidar_topics_[0], 1);
    sub2_.subscribe(nh_, lidar_topics_[1], 1);

    sync_.reset(new Sync(MySyncPolicy(10), sub1_, sub2_));
    sync_->registerCallback(boost::bind(&PCLMergeNode::pointCloudCallback, this, _1, _2));

    pub_ = nh_.advertise<sensor_msgs::PointCloud2>("merged_points", 1);

    ROS_INFO("PCLMergeNode subscribed to: %s and %s",
             lidar_topics_[0].c_str(), lidar_topics_[1].c_str());
}

void PCLMergeNode::pointCloudCallback(
    const sensor_msgs::PointCloud2ConstPtr& cloud1,
    const sensor_msgs::PointCloud2ConstPtr& cloud2)
{
    pcl::PointCloud<pcl::PointXYZ> pcl1, pcl2;
    pcl::fromROSMsg(*cloud1, pcl1);
    pcl::fromROSMsg(*cloud2, pcl2);

    pcl::PointCloud<pcl::PointXYZ> merged = pcl1;
    merged += pcl2;

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(merged, output);
    output.header.stamp = ros::Time::now();
    output.header.frame_id = cloud1->header.frame_id; // assuming same frame

    pub_.publish(output);
}
