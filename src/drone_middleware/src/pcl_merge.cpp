#include "drone_middleware/pcl_merge.hpp"

CloudMerger::CloudMerger(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : sub1_(nh, "", 10), sub2_(nh, "", 10),
    tf_buffer_(), tf_listener_(new tf2_ros::TransformListener(tf_buffer_))
{
  // parameters
  pnh.param<std::string>("cloud1_topic", cloud1_topic_, "/lidar_right/points");
  pnh.param<std::string>("cloud2_topic", cloud2_topic_, "/lidar_left/points");
  pnh.param<std::string>("output_topic", out_topic_, "/lidar/merged");
  pnh.param<std::string>("target_frame", target_frame_, std::string("")); // empty => no TF
  pnh.param("leaf_size", leaf_, 0.0);
  pnh.param("queue_size", queue_size_, 50);
  pnh.param("approx_slop", approx_slop_, 0.03);
  pnh.param("tf_timeout", tf_timeout_, 0.05);

  // wire subscribers & sync
  sub1_.subscribe(nh, cloud1_topic_, 10);
  sub2_.subscribe(nh, cloud2_topic_, 10);
  sync_.reset(new Sync(SyncPolicy(queue_size_), sub1_, sub2_));
  sync_->setMaxIntervalDuration(ros::Duration(approx_slop_));
  sync_->registerCallback(boost::bind(&CloudMerger::cb, this, _1, _2));

  pub_ = nh.advertise<sensor_msgs::PointCloud2>(out_topic_, 1, false);

  ROS_INFO_STREAM("[pcl_merge] cloud1_topic=" << cloud1_topic_
                  << " cloud2_topic=" << cloud2_topic_
                  << " output=" << out_topic_
                  << " target_frame=" << (target_frame_.empty() ? "<none>" : target_frame_)
                  << " leaf_size=" << leaf_);
}

bool CloudMerger::transformIfNeeded(const sensor_msgs::PointCloud2& in,
                                    sensor_msgs::PointCloud2& out)
{
  if (target_frame_.empty() || in.header.frame_id == target_frame_) {
    out = in;
    return true;
  }
  try {
    auto tf = tf_buffer_.lookupTransform(
        target_frame_, in.header.frame_id, in.header.stamp,
        ros::Duration(tf_timeout_));
    tf2::doTransform(in, out, tf);
    return true;
  } catch (const std::exception& e) {
    ROS_WARN_THROTTLE(1.0, "[pcl_merge] TF to %s failed from %s: %s",
                      target_frame_.c_str(), in.header.frame_id.c_str(), e.what());
    return false;
  }
}

void CloudMerger::cb(const sensor_msgs::PointCloud2ConstPtr& c1,
                     const sensor_msgs::PointCloud2ConstPtr& c2)
{
  // transform if requested
  sensor_msgs::PointCloud2 c1_tf, c2_tf;
  bool ok1 = transformIfNeeded(*c1, c1_tf);
  bool ok2 = transformIfNeeded(*c2, c2_tf);
  if (!ok1 || !ok2) return; // wait for TF to become available

  // to PCL
  pcl::PCLPointCloud2 pc1, pc2, merged;
  pcl_conversions::toPCL(c1_tf, pc1);
  pcl_conversions::toPCL(c2_tf, pc2);

  // concatenate (keeps fields/metadata)
  pcl::concatenatePointCloud(pc1, pc2, merged);

  // optional voxel downsample
  if (leaf_ > 1e-9) {
    pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
    pcl::PCLPointCloud2::Ptr merged_ptr(new pcl::PCLPointCloud2(merged));
    vg.setInputCloud(merged_ptr);
    vg.setLeafSize(leaf_, leaf_, leaf_);
    pcl::PCLPointCloud2 filtered;
    vg.filter(filtered);
    merged = filtered;
  }

  // back to ROS msg
  sensor_msgs::PointCloud2 out;
  pcl_conversions::fromPCL(merged, out);

  // header: frame is either target_frame_ (if set) or input frame
  out.header.frame_id = target_frame_.empty() ? c1_tf.header.frame_id : target_frame_;
  out.header.stamp = (c1_tf.header.stamp > c2_tf.header.stamp) ? c1_tf.header.stamp
                                                               : c2_tf.header.stamp;

  pub_.publish(out);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_merge_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  CloudMerger node(nh, pnh);
  ros::spin();
  return 0;
}
