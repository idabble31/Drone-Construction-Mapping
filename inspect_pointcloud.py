#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

# topics = ['/lidar_right/points','/lidar_left/points','/lidar/merged_points']
topics = ['/rslidar_front/points', 'cloud_registered', '/cloud_effected']

rospy.init_node('inspect_pc', anonymous=True)

for t in topics:
    try:
        msg = rospy.wait_for_message(t, PointCloud2, timeout=2.0)
    except Exception as e:
        print("NO MSG for", t, ":", e)
        continue
    
    print("=" * 80)
    print("=== TOPIC:", t)
    print("=" * 80)
    print("Frame ID:", msg.header.frame_id)
    print("Header Timestamp:", msg.header.stamp.to_sec())
    print("Point Step:", msg.point_step)
    print("Row Step:", msg.row_step)
    print("Width:", msg.width)
    print("Height:", msg.height)
    print("Total Points:", msg.width * msg.height)
    
    print("\n--- FIELDS ANALYSIS ---")
    print("Number of fields:", len(msg.fields))
    
    # Check for timestamp-related fields
    timestamp_fields = []
    field_names = []
    
    for f in msg.fields:
        field_names.append(f.name)
        print("  - Name: {:<15} Offset: {:<3} Datatype: {:<2} Count: {}".format(
            f.name, f.offset, f.datatype, f.count))
        
        # Check if this is a timestamp field
        if f.name.lower() in ['time', 'timestamp', 't', 'curvature']:
            timestamp_fields.append(f.name)
    
    print("\n--- TIMESTAMP CHECK ---")
    if timestamp_fields:
        print("✓ TIMESTAMP FIELDS FOUND:", timestamp_fields)
        print("  These fields can be used for per-point timestamps")
    else:
        print("✗ NO TIMESTAMP FIELD DETECTED")
        print("  Common timestamp field names: 'time', 'timestamp', 't', 'curvature'")
        print("  Available fields:", field_names)
    
    # Check specific fields needed by R3LIVE
    print("\n--- R3LIVE COMPATIBILITY CHECK ---")
    required_fields = ['x', 'y', 'z', 'intensity']
    optional_fields = ['curvature', 'time', 'timestamp', 'ring', 'normal_x', 'normal_y', 'normal_z']
    
    for rf in required_fields:
        if rf in field_names:
            print("✓", rf, "- PRESENT (required)")
        else:
            print("✗", rf, "- MISSING (required)")
    
    for of in optional_fields:
        if of in field_names:
            print("✓", of, "- PRESENT (optional)")
    
    # Special check for curvature field (R3LIVE uses this for timestamps)
    if 'curvature' in field_names:
        print("\n*** R3LIVE TIMESTAMP: 'curvature' field found!")
        print("    R3LIVE uses curvature field to store per-point timestamps")
    elif 'time' in field_names or 'timestamp' in field_names:
        print("\n*** TIMESTAMP FIELD: Found, but R3LIVE expects 'curvature'")
        print("    You may need to remap the timestamp field to 'curvature'")
    else:
        print("\n*** WARNING: No suitable timestamp field for R3LIVE!")
        print("    R3LIVE needs per-point timestamps in 'curvature' field")
    
    print("\n--- SAMPLE POINTS (First 5 points) ---")
    cnt = 0
    for p in pc2.read_points(msg, skip_nans=True):
        print("Point {}: {}".format(cnt+1, p))
        cnt += 1
        if cnt >= 5:
            break
    
    # Try to extract and analyze timestamp values if present
    if timestamp_fields:
        print("\n--- TIMESTAMP VALUES ANALYSIS ---")
        field_list = list(msg.fields)
        
        try:
            points_with_time = []
            for p in pc2.read_points(msg, field_names=timestamp_fields, skip_nans=True):
                points_with_time.append(p)
                if len(points_with_time) >= 100:  # Sample first 100 points
                    break
            
            if points_with_time:
                # Analyze timestamp field
                for ts_field in timestamp_fields:
                    ts_idx = timestamp_fields.index(ts_field)
                    values = [p[ts_idx] if isinstance(p, tuple) else p for p in points_with_time]
                    
                    min_val = min(values)
                    max_val = max(values)
                    
                    print("\nField '{}':".format(ts_field))
                    print("  Min value: {:.6f}".format(min_val))
                    print("  Max value: {:.6f}".format(max_val))
                    print("  Range: {:.6f}".format(max_val - min_val))
                    
                    # Determine if it's likely a timestamp
                    if max_val - min_val < 1.0 and max_val < 1.0:
                        print("  → Likely a RELATIVE timestamp (seconds from scan start)")
                        print("  → This is CORRECT format for R3LIVE!")
                    elif max_val > 1000000000:  # Unix timestamp
                        print("  → Appears to be ABSOLUTE Unix timestamp")
                        print("  → R3LIVE expects RELATIVE timestamps!")
                    else:
                        print("  → Unknown timestamp format")
                    
                    print("  First 5 values:", ["{:.6f}".format(v) for v in values[:5]])
        except Exception as e:
            print("Error analyzing timestamp values:", e)
    
    print("\n" + "=" * 80 + "\n")

print("\n=== SUMMARY ===")
print("Inspection complete!")
