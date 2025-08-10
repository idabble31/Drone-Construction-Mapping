#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/io.h>        // pcl::concatenatePointCloud
#include <pcl/filters/voxel_grid.h>

class TwoLidarMerger {
public:
  TwoLidarMerger(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : tf_buffer_(ros::Duration(10.0)), tf_listener_(tf_buffer_),
    sub1_(nh, pnh.param<std::string>("topic1", "/lidar_right/points"), 10),
    sub2_(nh, pnh.param<std::string>("topic2", "/lidar_left/points"), 10),
    sync_(SyncPolicy(pnh.param<int>("sync_queue", 10)), sub1_, sub2_) {

    out_topic_     = pnh.param<std::string>("output", "/merged/points");
    target_frame_  = pnh.param<std::string>("target_frame", std::string("")); // empty = use cloud1 frame
    leaf_size_     = pnh.param<double>("voxel_leaf", 0.0);                     // 0 = disabled
    transform_timeout_ = pnh.param<double>("tf_timeout", 0.05);

    pub_ = nh.advertise<sensor_msgs::PointCloud2>(out_topic_, 1);
    sync_.registerCallback(boost::bind(&TwoLidarMerger::callback, this, _1, _2));

    ROS_INFO_STREAM("[TwoLidarMerger] topic1=" << sub1_.getTopic()
                    << " topic2=" << sub2_.getTopic()
                    << " output=" << out_topic_
                    << " target_frame=" << (target_frame_.empty() ? "<cloud1 frame>" : target_frame_)
                    << " voxel_leaf=" << leaf_size_);
  }

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>;

  void callback(const sensor_msgs::PointCloud2ConstPtr& msg1,
                const sensor_msgs::PointCloud2ConstPtr& msg2) {
    try {
      // Decide target frame
      std::string tgt = target_frame_.empty() ? msg1->header.frame_id : target_frame_;

      // Transform both clouds into target frame (if needed)
      sensor_msgs::PointCloud2 c1_tf, c2_tf;
      if (msg1->header.frame_id != tgt) {
        if (!tf_buffer_.canTransform(tgt, msg1->header.frame_id, msg1->header.stamp,
                                     ros::Duration(transform_timeout_))) {
          ROS_WARN_THROTTLE(2.0, "TF missing for cloud1: %s -> %s", msg1->header.frame_id.c_str(), tgt.c_str());
          return;
        }
        tf2::doTransform(*msg1, c1_tf,
                         tf_buffer_.lookupTransform(tgt, msg1->header.frame_id,
                                                    msg1->header.stamp, ros::Duration(transform_timeout_)));
      } else {
        c1_tf = *msg1;
      }

      if (msg2->header.frame_id != tgt) {
        if (!tf_buffer_.canTransform(tgt, msg2->header.frame_id, msg2->header.stamp,
                                     ros::Duration(transform_timeout_))) {
          ROS_WARN_THROTTLE(2.0, "TF missing for cloud2: %s -> %s", msg2->header.frame_id.c_str(), tgt.c_str());
          return;
        }
        tf2::doTransform(*msg2, c2_tf,
                         tf_buffer_.lookupTransform(tgt, msg2->header.frame_id,
                                                    msg2->header.stamp, ros::Duration(transform_timeout_)));
      } else {
        c2_tf = *msg2;
      }

      // Convert to PCLPointCloud2 to preserve fields & concatenate
      pcl::PCLPointCloud2 pc1, pc2, merged;
      pcl_conversions::toPCL(c1_tf, pc1);
      pcl_conversions::toPCL(c2_tf, pc2);

      // Validate field layout
      if (pc1.fields.size() != pc2.fields.size()) {
        ROS_WARN_THROTTLE(2.0, "Field count mismatch (%zu vs %zu). Skipping this pair.",
                          pc1.fields.size(), pc2.fields.size());
        return;
      }

      pcl::concatenatePointCloud(pc1, pc2, merged);

      // Optional voxel grid downsample
      if (leaf_size_ > 1e-9) {
        pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
        pcl::PCLPointCloud2::Ptr merged_ptr(new pcl::PCLPointCloud2(merged)); // copy -> Ptr
        vg.setInputCloud(merged_ptr);
        vg.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
        pcl::PCLPointCloud2 filtered;
        vg.filter(filtered);
        merged = filtered; // assignment (some PCL versions lack swap() here)
      }


      // Back to ROS msg
      sensor_msgs::PointCloud2 out;
      pcl_conversions::fromPCL(merged, out);
      out.header.frame_id = tgt;
      // Choose a reasonable stamp (newest of the two inputs)
      out.header.stamp = (msg1->header.stamp > msg2->header.stamp) ? msg1->header.stamp : msg2->header.stamp;

      pub_.publish(out);
    }
    catch (const std::exception& e) {
      ROS_WARN_THROTTLE(2.0, "Exception in merge callback: %s", e.what());
    }
  }

  // ROS/TF
  ros::Publisher pub_;
  std::string out_topic_, target_frame_;
  double leaf_size_{0.0}, transform_timeout_{0.05};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  message_filters::Subscriber<sensor_msgs::PointCloud2> sub1_, sub2_;
  message_filters::Synchronizer<SyncPolicy> sync_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "merge_two_lidars");
  ros::NodeHandle nh, pnh("~");
  TwoLidarMerger node(nh, pnh);
  ros::spin();
  return 0;
}
