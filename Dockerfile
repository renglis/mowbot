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
    && rm -rf /var/lib/apt/lists/*

# 2) Install the spawner.py script into ROS's controller_manager path
RUN mkdir -p /opt/ros/humble/lib/controller_manager && \
    curl -o /opt/ros/humble/lib/controller_manager/spawner.py \
        https://raw.githubusercontent.com/ros-controls/ros2_control_demos/humble/controller_manager/scripts/spawner.py && \
    chmod +x /opt/ros/humble/lib/controller_manager/spawner.py

# 3) Copy local Jackal workspace (assumes user already cloned jackal, jackal_simulator, velodyne, etc.)
COPY jackal_ws /home/sim/jackal_ws

# 4) Patch Jackal URDF to avoid IMU plugin name conflict
RUN sed -i '/<plugin filename="libgazebo_ros_imu_sensor.so" name="\\$(arg prefix)imu_plugin">/a \
    <ros>\n      <node_name>imu_plugin_node</node_name>\n    </ros>' \
    /home/sim/jackal_ws/src/jackal/jackal_description/urdf/accessories/vlp16_mount.urdf.xacro

# 5) (Optional) Clone ros2_control_demos for reference
RUN git clone -b humble https://github.com/ros-controls/ros2_control_demos.git /home/sim/jackal_ws/src/ros2_control_demos

# 6) Create a non-root user 'sim' with sudo access
RUN useradd -ms /bin/bash sim && adduser sim sudo && echo "sim:sim" | chpasswd

# 7) Bake in ROS + workspace auto-sourcing; also create .ros and .gazebo to fix permission issues
RUN echo "source /opt/ros/humble/setup.bash" >> /home/sim/.bashrc && \
    echo "[ -f ~/jackal_ws/install/setup.bash ] && source ~/jackal_ws/install/setup.bash" >> /home/sim/.bashrc && \
    chown sim:sim /home/sim/.bashrc && \
    mkdir -p /home/sim/.ros && chown -R sim:sim /home/sim/.ros && \
    mkdir -p /home/sim/.gazebo && chown -R sim:sim /home/sim/.gazebo

# 8) Switch to user sim
USER sim
WORKDIR /home/sim

CMD ["bash"]
