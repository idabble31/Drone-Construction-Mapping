#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <string>
#include <ctime>

// Use Boost Filesystem (Standard in ROS) for robust directory handling
#include <boost/filesystem.hpp>

typedef pcl::PointXYZRGB PointType;

class MapSaverNode {
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_snapshot_;
    std::string save_dir_;

public:
    MapSaverNode() : nh_("~") {
        // 1. Get Parameters
        // Default to user's home directory if parameter is missing
        std::string home_path = getenv("HOME");
        nh_.param<std::string>("save_dir", save_dir_, home_path + "/scanhub_maps");

        // 2. Robust Directory Creation
        boost::filesystem::path dir(save_dir_);
        if (!boost::filesystem::exists(dir)) {
            ROS_WARN("[MapSaver] Directory '%s' does not exist. Creating it...", save_dir_.c_str());
            try {
                if (boost::filesystem::create_directories(dir)) {
                    ROS_INFO("[MapSaver] Successfully created directory.");
                } else {
                    ROS_ERROR("[MapSaver] Failed to create directory! Check permissions.");
                }
            } catch (const std::exception& e) {
                ROS_ERROR("[MapSaver] Directory creation crashed: %s", e.what());
            }
        }

        // 3. Subscriber
        sub_snapshot_ = nh_.subscribe("/r3live/map_snapshot_to_save", 1, &MapSaverNode::saveCallback, this);

        ROS_INFO("[MapSaver] Ready. Target Folder: %s", save_dir_.c_str());
    }

    void saveCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        ROS_WARN("[MapSaver] Received Map Snapshot! (%d points)", (int)(msg->width * msg->height));

        // 1. Generate Timestamped Filename
        std::time_t now = std::time(nullptr);
        char timestamp[100];
        std::strftime(timestamp, sizeof(timestamp), "%Y_%m_%d_%H_%M_%S", std::localtime(&now));
        
        // Use Boost to construct the path (handles slashes automatically)
        boost::filesystem::path dir(save_dir_);
        boost::filesystem::path file(std::string(timestamp) + "_scanhub_map.pcd");
        boost::filesystem::path full_path = dir / file;

        // 2. Convert ROS Msg -> PCL Cloud
        pcl::PointCloud<PointType> cloud;
        pcl::fromROSMsg(*msg, cloud);

        if (cloud.empty()) {
            ROS_ERROR("[MapSaver] Received empty cloud. Aborting save.");
            return;
        }

        // 3. Save to Disk
        std::string final_path_str = full_path.string();
        ROS_INFO("[MapSaver] Attempting to write to: %s", final_path_str.c_str());
        
        try {
            // Using Binary format
            int result = pcl::io::savePCDFileBinary(final_path_str, cloud);
            
            if (result == 0) {
                ROS_WARN("[MapSaver] SUCCESS! Saved %lu points.", cloud.size());
            } else {
                ROS_ERROR("[MapSaver] PCL returned error code %d", result);
            }
        } catch (const std::exception& e) {
            // This catches the specific "Error during open" exception
            ROS_ERROR("[MapSaver] EXCEPTION during save: %s", e.what());
            ROS_ERROR("[MapSaver] HINT: Does the folder '%s' exist? Do you have write permission?", save_dir_.c_str());
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_saver_node");
    MapSaverNode saver;
    ros::spin();
    return 0;
}