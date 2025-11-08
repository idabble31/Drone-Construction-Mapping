#pragma once


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/SetBool.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <mutex>
#include <string>


class PCDSaver {
public:
// nh should be a node handle in the desired namespace
PCDSaver(ros::NodeHandle& nh, const std::string &topic = "/points");
~PCDSaver();


// start listening (subscribes)
void start();
// stop listening and save last received cloud (if any)
void stop();


// service callback to toggle start/stop (SetBool: true->start, false->stop)
bool triggerService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);


bool isRunning() const;


private:
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
void saveLastCloud();


ros::NodeHandle nh_;
ros::Subscriber sub_;
ros::ServiceServer srv_;


std::string topic_;
std::string output_dir_;
std::string node_name_;


sensor_msgs::PointCloud2ConstPtr last_cloud_;
mutable std::mutex mutex_;
bool running_;
int file_index_;
};