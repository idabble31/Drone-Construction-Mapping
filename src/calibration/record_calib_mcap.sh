#!/bin/bash
set -euo pipefail

# Configuration
readonly BAG_DIR="${HOME}/calibration_bags"
readonly RECORD_DURATION=20

# Static Filenames for Docker Mounting
readonly STATIC_IMG="${BAG_DIR}/snap_target.jpg"

# Recording Output Directory
# We use a distinct path for mcap recording
readonly MCAP_BAG_NAME="calib_mcap"
readonly MCAP_OUTPUT_PATH="${BAG_DIR}/${MCAP_BAG_NAME}"

# Validate dependencies
command -v ros2 >/dev/null 2>&1 || { echo "Error: ros2 not found"; exit 1; }

mkdir -p "$BAG_DIR"

print_section() {
    echo "================================================"
    echo "$1"
    echo "================================================"
}

# 0. Backup previous attempt (just in case)
if [[ -d "$MCAP_OUTPUT_PATH" ]]; then
    rm -rf "${MCAP_OUTPUT_PATH}.last"
    mv "$MCAP_OUTPUT_PATH" "${MCAP_OUTPUT_PATH}.last"
    echo "Archived previous mcap recording to .last"
fi

if [[ -f "$STATIC_IMG" ]]; then
    rm -f "${STATIC_IMG}.last"
    mv "$STATIC_IMG" "${STATIC_IMG}.last"
    echo "Archived previous image to .last"
fi

# 1. Grab calibration image
print_section "Grabbing image for calibration..."
if ! ros2 run scan_routine fast_calib_image_grabber.py "$STATIC_IMG"; then
    echo "Error: Failed to capture calibration image" >&2
    exit 1
fi

# 2. Record topics
print_section "Starting ${RECORD_DURATION}-second Recording..."
# Remove old mcap if it exists (redundant with backup step, but for safety)
rm -rf "$MCAP_OUTPUT_PATH"

# Record directly to MCAP format
ros2 bag record -s mcap \
    -o "$MCAP_OUTPUT_PATH" \
    /image_raw /camera_info /rslidar_points /tf_static &

RECORD_PID=$!

# Wait for recording
sleep "$RECORD_DURATION"

# Graceful terminate
kill -INT "$RECORD_PID"
wait "$RECORD_PID" || true

print_section "SUCCESS!"
echo "Static Image: $STATIC_IMG"
echo "Static MCAP Path: $MCAP_OUTPUT_PATH"
