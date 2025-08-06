#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
import sys, select, termios, tty

def get_key():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        return sys.stdin.read(1)
    return ''

def main():
    rospy.init_node('teleop_attitude')
    pub = rospy.Publisher('/cmd_attitude', Float32MultiArray, queue_size=1)

    target = [0.0, 0.0, 0.0]  # roll, pitch, yaw

    print("Use WASD to change roll/pitch, QE to rotate (yaw), X to reset.")
    print("W/S: pitch forward/back\nA/D: roll left/right\nQ/E: yaw left/right")

    try:
        while not rospy.is_shutdown():
            key = get_key()
            if key == 'w': target[1] += 2.0  # pitch up
            elif key == 's': target[1] -= 2.0  # pitch down
            elif key == 'a': target[0] += 2.0  # roll left
            elif key == 'd': target[0] -= 2.0  # roll right
            elif key == 'q': target[2] += 2.0  # yaw left
            elif key == 'e': target[2] -= 2.0  # yaw right
            elif key == 'x': target = [0.0, 0.0, 0.0]  # reset

            msg = Float32MultiArray(data=target)
            pub.publish(msg)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))

if __name__ == '__main__':
    main()
