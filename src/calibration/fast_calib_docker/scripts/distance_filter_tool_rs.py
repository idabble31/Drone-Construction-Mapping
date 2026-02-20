#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FAST-Calib Target Lock Tool (Optimized for RoboSense)
1) Extracts 10 clean frames from /rslidar_points
2) Exports to PCD
3) Allows you to click the 4 holes/corners
4) Generates tight x/y/z bounds for your YAML
"""

import os
import sys
import numpy as np
import rosbag
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d

# ===================== PCD SAVING =====================

def save_pcd_with_intensity(points, intensities, output_path):
    N = len(points)
    header = f"""# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z intensity
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH {N}
HEIGHT 1
POINTS {N}
DATA ascii
"""
    with open(output_path, 'w') as f:
        f.write(header)
        for (x, y, z), inten in zip(points, intensities):
            f.write(f"{x} {y} {z} {inten}\n")
    print(f"[PCD] Saved point cloud with intensity to: {output_path}")

# ===================== POINTCLOUD2 (RoboSense) =====================

def find_intensity_field(msg):
    candidates = ["intensity", "reflectivity", "i", "ref"]
    for field in msg.fields:
        if field.name.lower() in candidates:
            return field.name
    return None

def convert_pointcloud2_bag_to_pcd(
    bag_file,
    output_dir,
    topic_name="/rslidar_points", # UPDATED FOR YOUR USE CASE
    pcd_name="target_lock_snapshot.pcd"
):
    print(f"[Bag] Opening rosbag: {bag_file}")
    bag = rosbag.Bag(bag_file, "r")

    intensity_field = None
    for topic, msg, t in bag.read_messages():
        if msg._type == "sensor_msgs/PointCloud2":
            intensity_field = find_intensity_field(msg)
            if intensity_field:
                print(f"[Bag] Detected intensity field: {intensity_field}")
            break

    if not intensity_field:
        print("[ERROR] No intensity field found!", file=sys.stderr)
        bag.close()
        return None

    all_points = []
    all_intensities = []
    count = 0
    max_frames = 10 # LIMITING TO 10 FRAMES FOR CLARITY

    print(f"[Bag] Reading {max_frames} frames from topic '{topic_name}'...")

    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        if count >= max_frames:
            break
        
        if msg._type == "sensor_msgs/PointCloud2":
            try:
                field_names = ["x", "y", "z", intensity_field]
                for point in pc2.read_points(msg, field_names=field_names, skip_nans=True):
                    all_points.append([point[0], point[1], point[2]])
                    all_intensities.append(point[3])
                count += 1
            except Exception as e:
                print(f"[ERROR] Read error: {str(e)}", file=sys.stderr)
                continue

    bag.close()

    if not all_points:
        print("[ERROR] No data found in /rslidar_points!", file=sys.stderr)
        return None

    output_path = os.path.join(output_dir, pcd_name)
    save_pcd_with_intensity(all_points, all_intensities, output_path)
    return output_path

# ===================== LIVOX (Left as fallback) =====================

def parse_livox_custom_msg(msg):
    points = []
    intensities = []
    for pt in msg.points:
        points.append([pt.x, pt.y, pt.z])
        intensities.append(pt.reflectivity)
    return points, intensities

def convert_livox_custom_bag_to_pcd(
    bag_file,
    output_dir,
    topic_name="/livox/lidar",
    pcd_name="livox_snapshot.pcd"
):
    bag = rosbag.Bag(bag_file, "r")
    all_points = []
    all_intensities = []
    count = 0

    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        if count >= 10: break
        if msg._type == "livox_ros_driver/CustomMsg":
            pts, intens = parse_livox_custom_msg(msg)
            all_points.extend(pts)
            all_intensities.extend(intens)
            count += 1

    bag.close()
    output_path = os.path.join(output_dir, pcd_name)
    save_pcd_with_intensity(all_points, all_intensities, output_path)
    return output_path

# ===================== AUTO-DETECT =====================

def detect_lidar_msg_type(bag_file):
    has_pc2 = False
    has_livox = False
    bag = rosbag.Bag(bag_file, "r")
    for topic, msg, t in bag.read_messages():
        if msg._type == "sensor_msgs/PointCloud2":
            has_pc2 = True
        elif msg._type == "livox_ros_driver/CustomMsg":
            has_livox = True
        if has_pc2 and has_livox: break
    bag.close()

    if has_pc2: return "PointCloud2"
    if has_livox: return "CustomMsg"
    return None

# ===================== INTERACTIVE PICKING & CROPPING =====================

def select_and_save_points(pcd_folder, target_pcd_name):
    pcd_path = os.path.join(pcd_folder, target_pcd_name)
    pcd = o3d.io.read_point_cloud(pcd_path)
    
    print(f"\n--- TARGET LOCK MODE ---")
    print("1. Use mouse to rotate/zoom.")
    print("2. Hold SHIFT + Left Click to pick the 4 target holes.")
    print("3. Press 'Q' to finish and generate YAML values.")

    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name=f"Pick Calibration Target - {target_pcd_name}")
    vis.add_geometry(pcd)
    vis.run()
    vis.destroy_window()

    selected_indices = vis.get_picked_points()

    if len(selected_indices) < 4:
        print("[ERROR] Pick at least 4 points!")
        return

    all_points = np.asarray(pcd.points)
    selected_points = all_points[selected_indices[:4], :]

    # CALCULATING BOUNDS
    mins = selected_points.min(axis=0)
    maxs = selected_points.max(axis=0)

    # TIGHT BUFFER: 0.05m (5cm) instead of 20cm
    buffer = 0.05
    x_min, x_max = mins[0] - buffer, maxs[0] + buffer
    y_min, y_max = mins[1] - buffer, maxs[1] + buffer
    z_min, z_max = mins[2] - buffer, maxs[2] + buffer

    save_file = os.path.join(pcd_folder, "target_lock_config.txt")
    with open(save_file, 'w') as f:
        f.write("# PASTE THESE INTO YOUR test_qr_params.yaml\n")
        f.write(f"x_min: {x_min:.3f}\n")
        f.write(f"x_max: {x_max:.3f}\n")
        f.write(f"y_min: {y_min:.3f}\n")
        f.write(f"y_max: {y_max:.3f}\n")
        f.write(f"z_min: {z_min:.3f}\n")
        f.write(f"z_max: {z_max:.3f}\n")

    print(f"\n[SUCCESS] Settings saved to: {save_file}")
    print(f"X: {x_min:.2f} to {x_max:.2f} | Y: {y_min:.2f} to {y_max:.2f} | Z: {z_min:.2f} to {z_max:.2f}")

if __name__ == "__main__":
    bag_file = sys.argv[1] if len(sys.argv) > 1 else "calib_target.bag"
    output_dir = sys.argv[2] if len(sys.argv) > 2 else os.getcwd()

    msg_type = detect_lidar_msg_type(bag_file)
    if msg_type == "PointCloud2":
        pcd_path = convert_pointcloud2_bag_to_pcd(bag_file, output_dir, topic_name="/rslidar_points")
    elif msg_type == "CustomMsg":
        pcd_path = convert_livox_custom_bag_to_pcd(bag_file, output_dir)
    else:
        print("No supported LiDAR data found.")
        sys.exit(1)

    if pcd_path:
        select_and_save_points(output_dir, os.path.basename(pcd_path))