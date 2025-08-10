#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/common/io.h>                 // pcl::concatenatePointCloud
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

class CloudMerger {
public:
  CloudMerger(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:
  // callback for synchronized clouds
  void cb(const sensor_msgs::PointCloud2ConstPtr& c1,
          const sensor_msgs::PointCloud2ConstPtr& c2);

  // (optional) transform cloud to target_frame (returns false if TF fails)
  bool transformIfNeeded(const sensor_msgs::PointCloud2& in,
                         sensor_msgs::PointCloud2& out);

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;

  // subs/pubs
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub1_, sub2_;
  std::shared_ptr<Sync> sync_;
  ros::Publisher pub_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  // params
  std::string cloud1_topic_;
  std::string cloud2_topic_;
  std::string out_topic_;
  std::string target_frame_;
  int queue_size_{50};
  double approx_slop_{0.03};
  double tf_timeout_{0.05};      // seconds
  double leaf_{0.0};             // voxel leaf size (m), 0 disables
};
