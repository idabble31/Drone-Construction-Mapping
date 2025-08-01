#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray

def publish_motor_speeds():
    pub = rospy.Publisher('/motor_speed', Float64MultiArray, queue_size=10)
    rospy.init_node('motor_speed_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10Hz
    speeds = Float64MultiArray(data=[100.0, 100.0, 100.0, 100.0]) # All props at 100 rad/s
    rospy.loginfo("Publishing motor speeds: %s", speeds.data)
    while not rospy.is_shutdown():
        pub.publish(speeds)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_motor_speeds()
    except rospy.ROSInterruptException:
        pass