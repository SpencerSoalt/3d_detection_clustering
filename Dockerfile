FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]

# ---- System deps ----
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-dev-tools \
    ros-humble-vision-msgs \
    tmux \
    vim \
    ros-humble-rosbag2 \
    ros-humble-rosbag2-storage-mcap \
    ros-humble-foxglove-bridge \
    ros-humble-rmw-cyclonedds-cpp \
    # PCL and point cloud dependencies
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    libpcl-dev \
    pcl-tools \
    # Velodyne driver support
    ros-humble-velodyne \
    ros-humble-velodyne-msgs \
    ros-humble-velodyne-pointcloud \
    # Visualization
    ros-humble-rviz2 \
    ros-humble-rviz-common \
    ros-humble-rviz-default-plugins \
    # Build tools
    build-essential \
    cmake \
    git \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/* 

# Set DDS/RMW defaults
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ---- Python deps for optional ML-based 3D detection ----
# RUN python3 -m pip install --no-cache-dir --upgrade pip wheel && \
#     python3 -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121 && \
#     python3 -m pip install --no-cache-dir open3d spconv-cu121 && \
#     python3 -m pip install --no-cache-dir --force-reinstall "numpy==1.26.4"

# Basic Python deps for point cloud processing
RUN python3 -m pip install --no-cache-dir --upgrade pip wheel && \
    python3 -m pip install --no-cache-dir \
    numpy \
    scipy \
    scikit-learn \
    open3d

# ---- ROS workspace ----
ENV WS=/ws
WORKDIR ${WS}

# Copy your package into the workspace
COPY src ${WS}/src

# rosdep (safe even if some keys already satisfied)
RUN rosdep init 2>/dev/null || true && rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# Build
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
