#!/usr/bin/env python3
import rospy
import csv
import os
from datetime import datetime

# Message types
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry, Path

class StabilityLogger:
    def __init__(self):
        rospy.init_node('stability_data_logger', anonymous=True)

        # 1. Setup CSV File
        # We create a filename with a timestamp so you don't overwrite old logs
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f"ros_stability_log_{timestamp_str}.csv"
        
        # Define the superset of columns
        self.header = [
            'timestamp', 'topic', 
            'pos_x', 'pos_y', 'pos_z',                # Position
            'orient_x', 'orient_y', 'orient_z', 'orient_w', # Orientation (Quat)
            'ang_vel_x', 'ang_vel_y', 'ang_vel_z',    # Angular Velocity
            'lin_acc_x', 'lin_acc_y', 'lin_acc_z'     # Linear Acceleration
        ]

        # Open the file and write the header immediately
        self.csv_file = open(self.filename, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(self.header)
        
        rospy.loginfo(f"Starting Logger. Saving to: {os.path.abspath(self.filename)}")

        # 2. Subscribers
        # We subscribe to the topics you requested
        self.sub_imu = rospy.Subscriber('/imu', Imu, self.imu_callback)
        self.sub_odom = rospy.Subscriber('/ins', Odometry, self.odom_callback)
        self.sub_lidar_imu = rospy.Subscriber('/rslidar_imu_data_front', Imu, self.lidar_imu_callback)
        self.sub_path = rospy.Subscriber('/path', Path, self.path_callback)

    def write_row(self, topic_name, timestamp, pos=None, orient=None, ang_vel=None, lin_acc=None):
        """
        Helper function to write a standardized row.
        None values will appear as empty cells in the CSV.
        """
        row = [timestamp, topic_name]
        
        # Position (x, y, z)
        row.extend([pos.x, pos.y, pos.z] if pos else ['', '', ''])
        
        # Orientation (x, y, z, w)
        row.extend([orient.x, orient.y, orient.z, orient.w] if orient else ['', '', '', ''])
        
        # Angular Velocity (x, y, z)
        row.extend([ang_vel.x, ang_vel.y, ang_vel.z] if ang_vel else ['', '', ''])
        
        # Linear Acceleration (x, y, z)
        row.extend([lin_acc.x, lin_acc.y, lin_acc.z] if lin_acc else ['', '', ''])

        self.writer.writerow(row)

    # --- Callbacks ---

    def imu_callback(self, msg):
        # IMU has Orientation, Angular Velocity, Linear Acceleration
        self.write_row(
            topic_name='/imu',
            timestamp=msg.header.stamp.to_sec(),
            orient=msg.orientation,
            ang_vel=msg.angular_velocity,
            lin_acc=msg.linear_acceleration
        )

    def lidar_imu_callback(self, msg):
        # Same structure as standard IMU
        self.write_row(
            topic_name='/rslidar_imu',
            timestamp=msg.header.stamp.to_sec(),
            orient=msg.orientation,
            ang_vel=msg.angular_velocity,
            lin_acc=msg.linear_acceleration
        )

    def odom_callback(self, msg):
        # Odom has Position and Orientation (in Pose)
        # It also has Twist (velocity), but usually, for stability, we check Pose or Twist
        # Here we log the Pose for trajectory comparison
        self.write_row(
            topic_name='/odom',
            timestamp=msg.header.stamp.to_sec(),
            pos=msg.pose.pose.position,
            orient=msg.pose.pose.orientation
        )

    def path_callback(self, msg):
        # Path is a generic list of poses.
        # To check stability, we usually look at the *latest* pose in the path 
        # to see if the global plan is jittering.
        if len(msg.poses) > 0:
            latest_pose = msg.poses[-1].pose
            self.write_row(
                topic_name='/path',
                timestamp=msg.header.stamp.to_sec(),
                pos=latest_pose.position,
                orient=latest_pose.orientation
            )

    def close(self):
        self.csv_file.close()
        rospy.loginfo("CSV file closed.")

if __name__ == '__main__':
    node = StabilityLogger()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        node.close()