# ROS 2 Humble + Gazebo Classic + Jackal Simulator Dockerfile for macOS
# Supports Apple Silicon or Intel via --platform linux/amd64
# X11-based or headless usage. Includes patches for spawner.py and IMU plugin.

FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# 1) Install required dependencies (includes Gazebo, controllers, etc.)
RUN apt-get update && apt-get install -y \
    git wget curl vim sudo net-tools locales \
    build-essential python3-colcon-common-extensions python3-pip \
    libpcap-dev \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-robot-localization \
    ros-humble-diff-drive-controller \
    ros-humble-ackermann-steering-controller \
    ros-humble-joint-state-broadcaster \
    ros-humble-controller-manager \
    ros-humble-controller-manager-msgs \
    ros-humble-control-msgs \
    ros-humble-twist-mux \
    ros-humble-imu-filter-madgwick \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-interactive-marker-twist-server \
    ros-humble-gazebo-ros2-control \
    x11-apps \
    gazebo \
    libgazebo-dev \
    unzip \
    && rm -rf /var/lib/apt/lists/*

# 2) Install the spawner.py script into ROS's controller_manager path
RUN mkdir -p /opt/ros/humble/lib/controller_manager && \
    curl -o /opt/ros/humble/lib/controller_manager/spawner.py \
        https://raw.githubusercontent.com/ros-controls/ros2_control_demos/humble/controller_manager/scripts/spawner.py && \
    chmod +x /opt/ros/humble/lib/controller_manager/spawner.py

# Patch shebang in spawner.py
RUN sed -i '1s|^.*$|#!/usr/bin/env python3|' /opt/ros/humble/lib/controller_manager/spawner.py && \
    chmod +x /opt/ros/humble/lib/controller_manager/spawner.py

# 3) Copy local Jackal workspace
COPY jackal_ws /root/jackal_ws

# 4) Patch Jackal URDF to avoid IMU plugin name conflict
RUN sed -i '/<plugin filename="libgazebo_ros_imu_sensor.so" name="\\$(arg prefix)imu_plugin">/a \
    <ros>\n      <node_name>imu_plugin_node</node_name>\n    </ros>' \
    /root/jackal_ws/src/jackal/jackal_description/urdf/accessories/vlp16_mount.urdf.xacro

# 5) (Optional) Clone ros2_control_demos for reference
RUN git clone -b humble https://github.com/ros-controls/ros2_control_demos.git /root/jackal_ws/src/ros2_control_demos

# 6) Install Gazebo models
RUN mkdir -p /usr/share/gazebo/models && \
    cd /usr/share/gazebo/models && \
    wget https://github.com/osrf/gazebo_models/archive/master.zip && \
    unzip master.zip && \
    mv gazebo_models-master/* . && \
    rm -rf gazebo_models-master master.zip && \
    chmod -R 755 /usr/share/gazebo/models

# 7) Set up environment variables
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "[ -f ~/jackal_ws/install/setup.bash ] && source ~/jackal_ws/install/setup.bash" >> /root/.bashrc && \
    echo "export GAZEBO_MODEL_PATH=/usr/share/gazebo/models:$GAZEBO_MODEL_PATH" >> /root/.bashrc

# 8) Install packages for VNC + minimal window manager
RUN apt-get update && apt-get install -y \
    x11vnc \
    xvfb \
    fluxbox \
    xterm && \
    rm -rf /var/lib/apt/lists/*

# 9) Copy startup script and set permissions
COPY start_vnc.sh /start_vnc.sh
RUN chmod +x /start_vnc.sh

# Expose VNC port
EXPOSE 5900

# Set working directory
WORKDIR /root

# Start VNC server
CMD ["/start_vnc.sh"]
