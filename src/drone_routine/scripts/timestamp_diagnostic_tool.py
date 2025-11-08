#!/usr/bin/env python3
"""
Timestamp Diagnostic Tool
Checks if sensor timestamps are compatible for synchronization
"""

import rospy
from sensor_msgs.msg import PointCloud2, Imu, Image

class TimestampChecker:
    def __init__(self):
        rospy.init_node('timestamp_checker')
        
        self.lidar_topic = rospy.get_param('~lidar_topic', '/rslidar_merged/points')
        self.imu_topic = rospy.get_param('~imu_topic', '/imu')
        self.camera_topic = rospy.get_param('~camera_topic', '/camera/image_color')
        
        self.lidar_time = None
        self.imu_time = None
        self.camera_time = None
        
        self.lidar_frame = None
        self.imu_frame = None
        self.camera_frame = None
        
        # Subscribe to each sensor
        rospy.Subscriber(self.lidar_topic, PointCloud2, self.lidar_callback)
        rospy.Subscriber(self.imu_topic, Imu, self.imu_callback)
        rospy.Subscriber(self.camera_topic, Image, self.camera_callback)
        
        rospy.loginfo("Timestamp Checker Started")
        rospy.loginfo(f"Listening to:")
        rospy.loginfo(f"  LiDAR: {self.lidar_topic}")
        rospy.loginfo(f"  IMU: {self.imu_topic}")
        rospy.loginfo(f"  Camera: {self.camera_topic}")
        rospy.loginfo("\nWaiting for messages...\n")
        
        # Timer to check periodically
        rospy.Timer(rospy.Duration(2.0), self.check_sync)
        
    def lidar_callback(self, msg):
        self.lidar_time = msg.header.stamp.to_sec()
        self.lidar_frame = msg.header.frame_id
        
    def imu_callback(self, msg):
        self.imu_time = msg.header.stamp.to_sec()
        self.imu_frame = msg.header.frame_id
        
    def camera_callback(self, msg):
        self.camera_time = msg.header.stamp.to_sec()
        self.camera_frame = msg.header.frame_id
        
    def check_sync(self, event):
        print("\n" + "="*80)
        print("TIMESTAMP DIAGNOSTIC REPORT")
        print("="*80)
        
        # Check if we received all messages
        if self.lidar_time is None:
            print("❌ NO LIDAR DATA RECEIVED")
        else:
            print(f"✓ LiDAR  : {self.lidar_time:.6f} (frame: {self.lidar_frame})")
            
        if self.imu_time is None:
            print("❌ NO IMU DATA RECEIVED")
        else:
            print(f"✓ IMU    : {self.imu_time:.6f} (frame: {self.imu_frame})")
            
        if self.camera_time is None:
            print("❌ NO CAMERA DATA RECEIVED")
        else:
            print(f"✓ Camera : {self.camera_time:.6f} (frame: {self.camera_frame})")
        
        print("-"*80)
        
        # Calculate differences if we have data
        if self.lidar_time and self.imu_time and self.camera_time:
            current_time = rospy.Time.now().to_sec()
            
            lidar_imu_diff = abs(self.lidar_time - self.imu_time) * 1000
            lidar_camera_diff = abs(self.lidar_time - self.camera_time) * 1000
            imu_camera_diff = abs(self.imu_time - self.camera_time) * 1000
            
            print(f"Time Differences:")
            print(f"  LiDAR ↔ IMU    : {lidar_imu_diff:>10.2f} ms")
            print(f"  LiDAR ↔ Camera : {lidar_camera_diff:>10.2f} ms")
            print(f"  IMU ↔ Camera   : {imu_camera_diff:>10.2f} ms")
            print("-"*80)
            
            # Check staleness
            lidar_age = (current_time - self.lidar_time) * 1000
            imu_age = (current_time - self.imu_time) * 1000
            camera_age = (current_time - self.camera_time) * 1000
            
            print(f"Message Ages (how old):")
            print(f"  LiDAR  : {lidar_age:>10.2f} ms old")
            print(f"  IMU    : {imu_age:>10.2f} ms old")
            print(f"  Camera : {camera_age:>10.2f} ms old")
            print("-"*80)
            
            # Diagnosis
            print("\nDIAGNOSIS:")
            
            max_diff = max(lidar_imu_diff, lidar_camera_diff, imu_camera_diff)
            
            if max_diff < 100:
                print("✓ GOOD: Timestamps are close enough for sync (< 100ms)")
                print("  Recommended slop: 0.1 seconds")
            elif max_diff < 500:
                print("⚠ MODERATE: Timestamps have some drift")
                print(f"  Recommended slop: {max_diff/1000 + 0.1:.2f} seconds")
            elif max_diff < 1000:
                print("⚠ LARGE: Timestamps are quite different")
                print(f"  Recommended slop: {max_diff/1000 + 0.2:.2f} seconds")
            else:
                print("❌ CRITICAL: Timestamps are very far apart!")
                print("  Possible issues:")
                print("    - Sensors using different time sources")
                print("    - One sensor not publishing hardware timestamps")
                print("    - Clock synchronization problem")
                print(f"  Try slop: {max_diff/1000 + 0.5:.2f} seconds (may not work)")
            
            # Check for zero timestamps
            if self.lidar_time == 0 or self.imu_time == 0 or self.camera_time == 0:
                print("\n❌ WARNING: Some timestamps are zero!")
                print("   Check if sensor drivers are configured to publish hardware timestamps")
            
            # Check staleness
            if max(lidar_age, imu_age, camera_age) > 1000:
                print("\n⚠ WARNING: Some messages are very old (> 1 second)")
                print("   Check if sensors are actually publishing regularly")
                
        else:
            print("⚠ Waiting for all sensor data...")
            print("\nTroubleshooting:")
            print("  1. Check if all sensors are running:")
            print(f"     rostopic hz {self.lidar_topic}")
            print(f"     rostopic hz {self.imu_topic}")
            print(f"     rostopic hz {self.camera_topic}")
            print("  2. Check topic names are correct")
            print("  3. Verify sensor drivers are launched")
        
        print("="*80)

if __name__ == '__main__':
    try:
        checker = TimestampChecker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass