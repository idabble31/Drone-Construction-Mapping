#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

def cmd_vel_callback(msg):
    target = Float32MultiArray()

    # Map cmd_vel to drone attitude setpoints
    roll = msg.linear.y      # left/right
    pitch = msg.linear.x     # forward/backward
    yaw = msg.angular.z      # yaw rotation
    throttle = msg.linear.z  # up/down

    # Optional: scale or clip values if needed
    target.data = [roll, pitch, yaw, throttle]
    pub.publish(target)

rospy.init_node('cmd_vel_to_att')
pub = rospy.Publisher('/cmd_attitude', Float32MultiArray, queue_size=1)
sub = rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)

rospy.loginfo("cmd_vel_to_att node started.")
rospy.spin()
