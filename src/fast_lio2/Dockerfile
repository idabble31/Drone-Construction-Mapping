# Use ros-base (better than core, includes standard libs)
# The official library supports ARM64 automatically.
FROM ros:noetic-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV CATKIN_WS=/root/catkin_ws

# 1. Install system dependencies
# (Your list was great, I just added 'wget' just in case)
RUN apt-get update && apt-get install -y \
    git \
    cmake \
    build-essential \
    libpcl-dev \
    python3-catkin-tools \
    nano \
    wget \
    ros-noetic-pcl-ros \
    ros-noetic-pcl-conversions \
    ros-noetic-cv-bridge \
    ros-noetic-tf \
    ros-noetic-message-generation \
    ros-noetic-eigen-conversions \
    && rm -rf /var/lib/apt/lists/*

# 2. Install Livox SDK (Required dependency for livox_ros_driver)
WORKDIR /root
# FIXED: Added "mkdir build" back in.
RUN git clone https://github.com/Livox-SDK/Livox-SDK.git && \
    cd Livox-SDK && \
    cd build && \
    cmake .. && make && make install && \
    cd ../.. && rm -rf Livox-SDK

# 3. Create Catkin Workspace
WORKDIR ${CATKIN_WS}/src

# 4. Clone FAST_LIO and the REQUIRED Livox Driver
# FIXED: Uncommented livox_ros_driver. FAST_LIO will NOT build without it.
RUN git clone https://github.com/hku-mars/FAST_LIO.git && \
    git clone https://github.com/Livox-SDK/livox_ros_driver.git && \
    cd FAST_LIO && \
    git submodule update --init

# 5. Build the workspace
WORKDIR ${CATKIN_WS}
# Source ros environment -> Build -> Refresh environment
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# 6. Add source command to .bashrc so it loads automatically
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source ${CATKIN_WS}/devel/setup.bash" >> /root/.bashrc

# 7. Set the entrypoint
ENTRYPOINT ["/bin/bash"]