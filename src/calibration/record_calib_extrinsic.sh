#!/bin/bash
set -euo pipefail

# Configuration
readonly BAG_DIR="${HOME}/calibration_bags"
readonly RECORD_DURATION=10

# Static Filenames for Docker Mounting
readonly STATIC_BAG="${BAG_DIR}/calib_target.bag"
readonly STATIC_IMG="${BAG_DIR}/snap_target.jpg"

# Temporary Recording Names
readonly MCAP_BAG_NAME="${BAG_DIR}/calib_mcap"

# Validate dependencies
command -v ros2 >/dev/null 2>&1 || { echo "Error: ros2 not found"; exit 1; }
command -v rosbags-convert >/dev/null 2>&1 || { echo "Error: rosbags-convert not found"; exit 1; }

mkdir -p "$BAG_DIR"

print_section() {
    echo "================================================"
    echo "$1"
    echo "================================================"
}

# 0. Backup previous attempt (just in case)
if [[ -f "$STATIC_BAG" ]]; then
    mv "$STATIC_BAG" "${STATIC_BAG}.last"
    mv "$STATIC_IMG" "${STATIC_IMG}.last"
    echo "Archived previous files to .last"
fi

# 1. Grab calibration image
print_section "Grabbing image for calibration..."
if ! ros2 run scan_routine fast_calib_image_grabber.py "$STATIC_IMG"; then
    echo "Error: Failed to capture calibration image" >&2
    exit 1
fi

# 2. Record topics
print_section "Starting ${RECORD_DURATION}-second Recording..."
# Remove old tmp mcap if it exists
rm -rf "${BAG_DIR}/${MCAP_BAG_NAME}"

ros2 bag record -s mcap \
    -o "${BAG_DIR}/${MCAP_BAG_NAME}" \
    /image_raw /camera_info /rslidar_points /tf_static &

RECORD_PID=$!
sleep "$RECORD_DURATION"
kill -INT "$RECORD_PID"
sleep 2

# 3. Convert to ROS1 format using the STATIC name
print_section "Converting to ROS 1 format..."
if ! rosbags-convert --src "${BAG_DIR}/${MCAP_BAG_NAME}" --dst "$STATIC_BAG"; then
    echo "Error: Conversion failed" >&2
    exit 1
fi

# 4. Cleanup MCAP
rm -rf "${BAG_DIR}/${MCAP_BAG_NAME}"

print_section "SUCCESS!"
echo "Static Image: $STATIC_IMG"
echo "Static Bag:   $STATIC_BAG"