#!/usr/bin/env python3
"""
Timestamp Correction Relay
Republishes IMU messages with corrected timestamps
"""

import rospy
from sensor_msgs.msg import Imu
from collections import deque

class TimestampRelay:
    def __init__(self):
        rospy.init_node('timestamp_relay')
        
        # Parameters
        self.imu_input_topic = rospy.get_param('~imu_input_topic', '/imu')
        self.imu_output_topic = rospy.get_param('~imu_output_topic', '/imu_corrected')
        self.time_offset = rospy.get_param('~time_offset', 0.0)  # seconds to subtract
        self.auto_offset = rospy.get_param('~auto_offset', True)  # auto-calculate offset
        
        # For auto offset calculation
        self.offset_samples = deque(maxlen=50)
        self.calculated_offset = 0.0
        self.offset_ready = False
        
        # Publisher and Subscriber
        self.pub = rospy.Publisher(self.imu_output_topic, Imu, queue_size=10)
        self.sub = rospy.Subscriber(self.imu_input_topic, Imu, self.callback)
        
        rospy.loginfo("Timestamp Relay Node Started")
        rospy.loginfo(f"Input:  {self.imu_input_topic}")
        rospy.loginfo(f"Output: {self.imu_output_topic}")
        
        if self.auto_offset:
            rospy.loginfo("Auto-offset mode: Calculating time offset...")
        else:
            rospy.loginfo(f"Manual offset: {self.time_offset:.3f} seconds")
    
    def callback(self, msg):
        # Calculate offset if in auto mode
        if self.auto_offset and not self.offset_ready:
            current_time = rospy.Time.now().to_sec()
            msg_time = msg.header.stamp.to_sec()
            offset = msg_time - current_time
            
            self.offset_samples.append(offset)
            
            if len(self.offset_samples) >= 50:
                self.calculated_offset = sum(self.offset_samples) / len(self.offset_samples)
                self.offset_ready = True
                rospy.loginfo(f"✓ Offset calculated: {self.calculated_offset:.3f} seconds")
                rospy.loginfo(f"  (IMU is {abs(self.calculated_offset)*1000:.1f}ms {'ahead' if self.calculated_offset > 0 else 'behind'})")
            else:
                # Still calculating, don't publish yet
                return
        
        # Create corrected message
        corrected_msg = msg
        
        # Apply offset
        offset_to_use = self.calculated_offset if self.auto_offset else self.time_offset
        original_time = msg.header.stamp.to_sec()
        corrected_time = original_time - offset_to_use
        
        corrected_msg.header.stamp = rospy.Time.from_sec(corrected_time)
        
        # Publish
        self.pub.publish(corrected_msg)

if __name__ == '__main__':
    try:
        relay = TimestampRelay()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
