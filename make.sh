#!/bin/bash

# 1. Source the ROS 2 system underlay (Mandatory for package registration)
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
else
    echo "Error: ROS 2 Humble not found at /opt/ros/humble/setup.bash"
    exit 1
fi

# 2. Optional: Clean old build/install artifacts to prevent 'Package not found' errors
# If you keep having the 'Package not found' issue, uncomment the next line:
# rm -rf build/ install/ log/

# 3. Run colcon build
colcon build \
        --symlink-install \
        --continue-on-error \
        --cmake-args \
        -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
        -DUSE_LIVOX=OFF \
        -DENABLE_IMU_DATA_PARSE=ON \
        -DCMAKE_BUILD_TYPE=Release

# 4. Source the local workspace so the current terminal recognizes the new build
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo "Workspace sourced successfully."
else
    echo "Warning: install/setup.bash not found. Build might have failed."
fi