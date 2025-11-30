/* 
This code is the implementation of our paper "R3LIVE: A Robust, Real-time, RGB-colored, 
LiDAR-Inertial-Visual tightly-coupled state Estimation and mapping package".

Author: Jiarong Lin   < ziv.lin.ljr@gmail.com >

If you use any code of this repo in your academic research, please cite at least
one of our papers:
[1] Lin, Jiarong, and Fu Zhang. "R3LIVE: A Robust, Real-time, RGB-colored, 
    LiDAR-Inertial-Visual tightly-coupled state Estimation and mapping package." 
[2] Xu, Wei, et al. "Fast-lio2: Fast direct lidar-inertial odometry."
[3] Lin, Jiarong, et al. "R2LIVE: A Robust, Real-time, LiDAR-Inertial-Visual
     tightly-coupled state Estimator and mapping." 
[4] Xu, Wei, and Fu Zhang. "Fast-lio: A fast, robust lidar-inertial odometry 
    package by tightly-coupled iterated kalman filter."
[5] Cai, Yixi, Wei Xu, and Fu Zhang. "ikd-Tree: An Incremental KD Tree for 
    Robotic Applications."
[6] Lin, Jiarong, and Fu Zhang. "Loam-livox: A fast, robust, high-precision 
    LiDAR odometry and mapping package for LiDARs of small FoV."

For commercial use, please contact me < ziv.lin.ljr@gmail.com > and
Dr. Fu Zhang < fuzhang@hku.hk >.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.
 2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
 3. Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from this
    software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
*/
#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <so3_math.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <common_lib.h>
#include <kd_tree/ikd_Tree.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <opencv2/core/eigen.hpp>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <FOV_Checker/FOV_Checker.h>

#include "r3live.hpp"

#include "loam/IMU_Processing.hpp"
#include "tools_logger.hpp"
#include "tools_color_printf.hpp"
#include "tools_eigen.hpp"
#include "tools_data_io.hpp"
#include "tools_timer.hpp"
#include "tools_openCV_3_to_4.hpp"

Camera_Lidar_queue g_camera_lidar_queue;
MeasureGroup Measures;
StatesGroup g_lio_state;
std::string data_dump_dir = std::string("/mnt/0B3B134F0B3B134F/color_temp_r3live/");

void R3LIVE::execute_action_cb(const drone_middleware::ScanMapGoalConstPtr &goal)
{
    ros::Rate r(10); // Feedback rate (10Hz)
    
    // --- 1. HANDLE COMMANDS ---
    if (goal->command == 1) { // START
        ROS_WARN("[ScanHub] Mapping STARTED");
        is_mapping_active = true;
        is_paused = false;
        // Optional: Reset map here if needed
    }
    else if (goal->command == 2) { // PAUSE
        ROS_WARN("[ScanHub] Mapping PAUSED");
        is_paused = true;
    }
    else if (goal->command == 3) { // RESUME
        ROS_WARN("[ScanHub] Mapping RESUMED");
        is_paused = false;
    }

    // --- 2. CONTROL LOOP ---
    while(ros::ok()) {
        
        // CHECK IF CLIENT CANCELLED (STOP/SAVE)
        if (as_->isPreemptRequested() || !ros::ok()) {
            ROS_WARN("[ScanHub] Stop & Save Requested...");
            
            // A. Close the Gate
            is_mapping_active = false;
            
            // B. Prepare Filename
            std::string filename = goal->filename;
            if (filename.empty()) filename = "scanhub_map";
            std::string full_path = m_map_output_dir + "/" + filename + ".pcd";

            // C. Save the Map (THREAD SAFE LOCK)
            m_mutex_lio_process.lock();
            if (featsFromMap->points.empty()) {
                ROS_WARN("Map is empty, nothing to save!");
            } else {
                pcl::io::savePCDFileBinary(full_path, *featsFromMap);
                ROS_WARN("Map Saved to: %s", full_path.c_str());
            }
            m_mutex_lio_process.unlock();

            // D. Send Result
            result_.success = true;
            result_.filepath = full_path;
            as_->setSucceeded(result_);
            break; // Exit the action loop
        }

        // --- 3. SEND FEEDBACK ---
        if (featsFromMap) {
            feedback_.point_count = featsFromMap->points.size();
        }
        // Assuming g_camera_frame_idx tracks frames (from r3live.hpp)
        feedback_.frame_count = g_camera_frame_idx; 
        feedback_.current_state = is_paused ? 2 : (is_mapping_active ? 1 : 0);
        
        as_->publishFeedback(feedback_);
        r.sleep();
    }
}

int main(int argc, char **argv)
{
    printf_program("R3LIVE: A Robust, Real-time, RGB-colored, LiDAR-Inertial-Visual tightly-coupled state Estimation and mapping package");
    Common_tools::printf_software_version();
    Eigen::initParallel();
    ros::init(argc, argv, "R3LIVE_main");
    R3LIVE * fast_lio_instance = new R3LIVE();
    ros::Rate rate(5000);
    bool status = ros::ok();
    ros::spin();
}
