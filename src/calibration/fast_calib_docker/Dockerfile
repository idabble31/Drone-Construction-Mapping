# Use the base image that supports arm64
FROM arm64v8/ros:noetic-ros-core-focal

# Set environment variables to avoid prompts during install
ENV DEBIAN_FRONTEND=noninteractive

# Install desktop components and FAST-Calib dependencies
# Added: ros-noetic-vision-opencv (contains image_geometry)
RUN apt-get update && apt-get install -y \
    ros-noetic-desktop \
    ros-noetic-pcl-ros \
    ros-noetic-opencv-apps \
    ros-noetic-vision-opencv \
    git \
    nano \
    python3-pip \
    python3-catkin-tools \
    libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

# Upgrade pip & install Open3D
RUN pip3 install --upgrade pip && \
    pip3 install open3d

# Create workspace
WORKDIR /catkin_ws/src
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; catkin init"

# Clone FAST-Calib
RUN git clone https://github.com/hku-mars/FAST-Calib.git

# Build
WORKDIR /catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; catkin build"

# Add setup to bashrc
RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

ENTRYPOINT ["/bin/bash"]
