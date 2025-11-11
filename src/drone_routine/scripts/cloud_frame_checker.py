#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2

def pointcloud_callback(msg):
    """Callback function that prints the frame_id from PointCloud2 message"""
    rospy.loginfo(f"PointCloud2 Frame ID: {msg.header.frame_id}")
    rospy.loginfo(f"Timestamp: {msg.header.stamp.to_sec()}")
    rospy.loginfo("---")

def main():
    # Initialize the ROS node
    rospy.init_node('pointcloud_frame_checker', anonymous=True)
    
    # Get the topic name from parameter or use default
    topic_name = rospy.get_param('~topic', '/cloud_registered')
    
    rospy.loginfo(f"Subscribing to topic: {topic_name}")
    
    # Subscribe to the PointCloud2 topic
    rospy.Subscriber(topic_name, PointCloud2, pointcloud_callback)
    
    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass