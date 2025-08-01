#!/usr/bin/env python3
import rospy, sys, select, termios, tty
from std_msgs.msg import Float64MultiArray

# Key → (Δ front_right, Δ front_left, Δ rear_left, Δ rear_right)
bindings = {
    'w': ( 10,  10,  10,  10),   # all props up
    's': (-10, -10, -10, -10),   # all props down
    'i': ( 10, -10, -10,  10),   # pitch forward
    'k': (-10,  10,  10, -10),   # pitch back
    'j': (-10, -10,  10,  10),   # roll left
    'l': ( 10,  10, -10, -10),   # roll right
    'a': ( 10, -10,  10, -10),   # yaw left
    'd': (-10,  10, -10,  10),   # yaw right
}

msg = """
Controls:
  w/s : collective throttle
  i/k : pitch forward/back
  j/l : roll left/right
  a/d : yaw left/right
CTRL-C to quit
"""

def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        if select.select([sys.stdin], [], [], 0.1)[0]:
            return sys.stdin.read(1)
        return ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

def teleop():
    rospy.init_node('drone_teleop')
    pub = rospy.Publisher('/motor_speed', Float64MultiArray, queue_size=1)
    settings = termios.tcgetattr(sys.stdin)
    speeds = [0.0, 0.0, 0.0, 0.0]

    try:
        print(msg)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            key = get_key()
            if key == '\x03':  # Ctrl+C
                break

            if key in bindings:
                delta = bindings[key]
                # increment each rotor’s speed
                speeds = [max(0.0, sp + d) for sp, d in zip(speeds, delta)]

            # publish array of four speeds
            pub.publish(Float64MultiArray(data=speeds))
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    finally:
        # zero them out on exit
        pub.publish(Float64MultiArray(data=[0,0,0,0]))
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    teleop()
