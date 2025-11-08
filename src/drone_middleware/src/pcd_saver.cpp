#include "drone_middleware/pcd_saver.hpp"
#include <ros/ros.h>
#include <boost/filesystem.hpp>
#include <iomanip>

PCDSaver::PCDSaver(ros::NodeHandle &nh, const std::string &topic)
  : nh_(nh), topic_(topic), running_(false), file_index_(0)
{
  nh_.param<std::string>("output_dir", output_dir_, "./pcd_output");
  nh_.param<std::string>("node_name", node_name_, std::string("pcd_saver"));

  // create output dir if missing
  if (!boost::filesystem::exists(output_dir_)) {
    boost::filesystem::create_directories(output_dir_);
  }

  // service to trigger start/stop
  srv_ = nh_.advertiseService("trigger", &PCDSaver::triggerService, this);

  ROS_INFO("[%s] Ready. use service '%s/trigger' (SetBool) to start/stop.", node_name_.c_str(), nh_.getNamespace().c_str());
}

PCDSaver::~PCDSaver()
{
  if (running_) stop();
}

void PCDSaver::start()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (running_) return;
  sub_ = nh_.subscribe(topic_, 1, &PCDSaver::cloudCallback, this);
  running_ = true;
  ROS_INFO("[%s] Subscribed to '%s'.", node_name_.c_str(), topic_.c_str());
}

void PCDSaver::stop()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_) return;
    sub_.shutdown();
    running_ = false;
  }
  ROS_INFO("[%s] Unsubscribed from '%s' and saving last cloud...", node_name_.c_str(), topic_.c_str());
  saveLastCloud();
}

bool PCDSaver::isRunning() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return running_;
}

void PCDSaver::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  // store the most recent cloud thread-safely
  std::lock_guard<std::mutex> lock(mutex_);
  last_cloud_ = msg;
}

void PCDSaver::saveLastCloud()
{
  sensor_msgs::PointCloud2ConstPtr cloud_copy;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cloud_copy = last_cloud_;
  }

  if (!cloud_copy) {
    ROS_WARN("[%s] No pointcloud received to save.", node_name_.c_str());
    return;
  }

  // convert to PCL PCLPointCloud2
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*cloud_copy, pcl_pc2);

  // build filename with timestamp and index
  ros::Time t = cloud_copy->header.stamp;
  std::ostringstream ss;
  ss << output_dir_ << "/pcd_" << t.sec << "_" << std::setw(6) << std::setfill('0') << t.nsec << "_" << file_index_ << ".pcd";
  std::string filename = ss.str();

  // Try to convert to a templated PointCloud<PointXYZ> so we can use
  // savePCDFileBinaryCompressed (which is templated). If conversion
  // yields no points, fall back to saving the raw PCLPointCloud2 with
  // savePCDFile (uncompressed).
  pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
  try {
    pcl::fromPCLPointCloud2(pcl_pc2, cloud_xyz);
  } catch (const std::exception &e) {
    ROS_WARN("[%s] fromPCLPointCloud2 threw: %s", node_name_.c_str(), e.what());
  }

  if (!cloud_xyz.empty()) {
    if (pcl::io::savePCDFileBinaryCompressed(filename, cloud_xyz) == 0) {
      ROS_INFO("[%s] Saved PCD '%s' (compressed)", node_name_.c_str(), filename.c_str());
      file_index_++;
      return;
    } else {
      ROS_ERROR("[%s] Failed to save compressed PCD '%s'", node_name_.c_str(), filename.c_str());
    }
  }

  // Fallback: save raw PCLPointCloud2 (may be uncompressed ASCII/BINARY depending on savePCDFile implementation)
  if (pcl::io::savePCDFile(filename, pcl_pc2) == 0) {
    ROS_INFO("[%s] Saved PCD '%s' (raw PCLPointCloud2)", node_name_.c_str(), filename.c_str());
    file_index_++;
  } else {
    ROS_ERROR("[%s] Failed to save PCD '%s' (fallback)", node_name_.c_str(), filename.c_str());
  }
}


bool PCDSaver::triggerService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  if (req.data) {
    start();
    res.success = true;
    res.message = "Started listening";
  } else {
    stop();
    res.success = true;
    res.message = "Stopped listening and saved last cloud";
  }
  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pcd_saver_node");
  ros::NodeHandle nh("~"); // private namespace for params

  std::string topic;
  nh.param<std::string>("topic", topic, std::string("/points"));

  PCDSaver saver(nh, topic);

  // keep spinning to service trigger and accept callbacks when started
  ros::spin();
  return 0;
}
