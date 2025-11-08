#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

# topics = ['/lidar_right/points','/lidar_left/points','/lidar/merged_points']
topics = ['/rslidar_front/points','/rslidar_rear/points','/rslidar_merged/points']
rospy.init_node('inspect_pc', anonymous=True)
for t in topics:
    try:
        msg = rospy.wait_for_message(t, PointCloud2, timeout=2.0)
    except Exception as e:
        print("NO MSG for", t, ":", e); continue
    print("=== TOPIC:", t, " frame:", msg.header.frame_id, " stamp:", msg.header.stamp.to_sec())
    print("point_step:", msg.point_step, "row_step:", msg.row_step, "width:", msg.width, "height:", msg.height)
    print("fields:", [ (f.name,f.offset,f.datatype,f.count) for f in msg.fields ])
    print("first up to 5 points (fields in order):")
    cnt=0
    for p in pc2.read_points(msg, skip_nans=True):
        print(p)
        cnt+=1
        if cnt>=5: break
    print("\n")
