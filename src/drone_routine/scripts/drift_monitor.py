#!/usr/bin/env python3
import rospy
import tf2_ros
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class DriftMonitor:
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        
        self.positions_x = []
        self.positions_y = []
        self.times = []
        self.start_time = rospy.Time.now()
        
    def update(self, frame):
        try:
            trans = self.tfBuffer.lookup_transform(
                'world', 'base_link', rospy.Time(0))
            
            elapsed = (rospy.Time.now() - self.start_time).to_sec()
            
            self.times.append(elapsed)
            self.positions_x.append(trans.transform.translation.x)
            self.positions_y.append(trans.transform.translation.y)
            
            # Plot
            plt.clf()
            plt.subplot(2, 1, 1)
            plt.plot(self.times, self.positions_x, 'r-', label='X position')
            plt.ylabel('X Position (m)')
            plt.title('Drift Monitor - Robot Should Be STATIONARY')
            plt.grid(True)
            plt.legend()
            
            plt.subplot(2, 1, 2)
            plt.plot(self.times, self.positions_y, 'b-', label='Y position')
            plt.xlabel('Time (s)')
            plt.ylabel('Y Position (m)')
            plt.grid(True)
            plt.legend()
            
            # Calculate drift rate
            if len(self.times) > 1:
                total_drift = ((self.positions_x[-1]**2 + 
                               self.positions_y[-1]**2)**0.5)
                drift_rate = total_drift / elapsed
                plt.suptitle(f'Total Drift: {total_drift:.3f}m, '
                           f'Rate: {drift_rate:.4f}m/s')
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException):
            pass

if __name__ == '__main__':
    rospy.init_node('drift_monitor')
    
    monitor = DriftMonitor()
    
    fig = plt.figure(figsize=(10, 8))
    ani = FuncAnimation(fig, monitor.update, interval=100)
    
    plt.show()
    rospy.spin()