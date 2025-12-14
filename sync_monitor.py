#!/usr/bin/env python3
"""
Sensor Synchronization Monitor
Monitors and displays timestamp differences between LiDAR, IMU, and Camera
"""

import rospy
from sensor_msgs.msg import PointCloud2, Imu, Image
import message_filters
from collections import deque
import numpy as np
import os

class SyncMonitor:
    def __init__(self):
        rospy.init_node('sync_monitor')
        
        # Get topic names from parameters
        self.lidar_topic = rospy.get_param('~lidar_topic', '/rslidar_front/points')
        self.imu_topic = rospy.get_param('~imu_topic', '/imu')
        self.camera_topic = rospy.get_param('~camera_topic', '/front_camera/image_dewarped')
        
        # Statistics tracking
        self.lidar_imu_diffs = deque(maxlen=100)
        self.lidar_camera_diffs = deque(maxlen=100)
        self.imu_camera_diffs = deque(maxlen=100)
        
        self.msg_count = 0
        
        # Subscribers
        self.lidar_sub = message_filters.Subscriber(self.lidar_topic, PointCloud2)
        self.imu_sub = message_filters.Subscriber(self.imu_topic, Imu)
        self.camera_sub = message_filters.Subscriber(self.camera_topic, Image)
        
        # Synchronizer
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.lidar_sub, self.imu_sub, self.camera_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.callback)
        
        # Timer for display update
        self.display_timer = rospy.Timer(rospy.Duration(1.0), self.display_stats)
        
        rospy.loginfo("Sync Monitor Started")
        rospy.loginfo(f"LiDAR topic: {self.lidar_topic}")
        rospy.loginfo(f"IMU topic: {self.imu_topic}")
        rospy.loginfo(f"Camera topic: {self.camera_topic}")
        
    def callback(self, lidar_msg, imu_msg, camera_msg):
        # Get timestamps in seconds
        lidar_time = lidar_msg.header.stamp.to_sec()
        imu_time = imu_msg.header.stamp.to_sec()
        camera_time = camera_msg.header.stamp.to_sec()
        
        # Calculate differences in milliseconds
        lidar_imu_diff = abs(lidar_time - imu_time) * 1000
        lidar_camera_diff = abs(lidar_time - camera_time) * 1000
        imu_camera_diff = abs(imu_time - camera_time) * 1000
        
        # Store differences
        self.lidar_imu_diffs.append(lidar_imu_diff)
        self.lidar_camera_diffs.append(lidar_camera_diff)
        self.imu_camera_diffs.append(imu_camera_diff)
        
        self.msg_count += 1
    
    def display_stats(self, event):
        if len(self.lidar_imu_diffs) == 0:
            rospy.logwarn("No synchronized messages received yet...")
            return
        
        # Clear terminal
        os.system('clear' if os.name == 'posix' else 'cls')
        
        # Calculate statistics
        li_mean = np.mean(self.lidar_imu_diffs)
        li_max = np.max(self.lidar_imu_diffs)
        li_min = np.min(self.lidar_imu_diffs)
        li_std = np.std(self.lidar_imu_diffs)
        
        lc_mean = np.mean(self.lidar_camera_diffs)
        lc_max = np.max(self.lidar_camera_diffs)
        lc_min = np.min(self.lidar_camera_diffs)
        lc_std = np.std(self.lidar_camera_diffs)
        
        ic_mean = np.mean(self.imu_camera_diffs)
        ic_max = np.max(self.imu_camera_diffs)
        ic_min = np.min(self.imu_camera_diffs)
        ic_std = np.std(self.imu_camera_diffs)
        
        # Status indicators
        def get_status(mean_diff):
            if mean_diff < 10:
                return "✓ EXCELLENT"
            elif mean_diff < 30:
                return "✓ GOOD"
            elif mean_diff < 50:
                return "⚠ ACCEPTABLE"
            else:
                return "✗ POOR"
        
        # Print table
        print("=" * 90)
        print("SENSOR SYNCHRONIZATION MONITOR".center(90))
        print("=" * 90)
        print(f"Total Synchronized Messages: {self.msg_count}")
        print(f"Sample Size: {len(self.lidar_imu_diffs)} messages")
        print("-" * 90)
        
        # Header
        print(f"{'Sensor Pair':<20} {'Mean (ms)':<12} {'Min (ms)':<12} {'Max (ms)':<12} {'Std (ms)':<12} {'Status':<12}")
        print("-" * 90)
        
        # LiDAR-IMU
        print(f"{'LiDAR ↔ IMU':<20} {li_mean:>10.2f}  {li_min:>10.2f}  {li_max:>10.2f}  {li_std:>10.2f}  {get_status(li_mean):<12}")
        
        # LiDAR-Camera
        print(f"{'LiDAR ↔ Camera':<20} {lc_mean:>10.2f}  {lc_min:>10.2f}  {lc_max:>10.2f}  {lc_std:>10.2f}  {get_status(lc_mean):<12}")
        
        # IMU-Camera
        print(f"{'IMU ↔ Camera':<20} {ic_mean:>10.2f}  {ic_min:>10.2f}  {ic_max:>10.2f}  {ic_std:>10.2f}  {get_status(ic_mean):<12}")
        
        print("=" * 90)
        print("\nSynchronization Quality Guide:")
        print("  < 10ms  : Excellent - Suitable for high-precision applications")
        print("  < 30ms  : Good - Suitable for most SLAM applications")
        print("  < 50ms  : Acceptable - May work for some applications")
        print("  > 50ms  : Poor - Synchronization issues likely")
        print("\nPress Ctrl+C to stop monitoring")
        print("=" * 90)
        
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        monitor = SyncMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        pass
