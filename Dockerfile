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
    ros-humble-imu-tools \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-interactive-marker-twist-server \
    ros-humble-gazebo-ros2-control \
    ros-humble-sensor-msgs \
    ros-humble-gazebo-dev \
    ros-humble-gazebo-ros \
    ros-humble-std-msgs \
    ros-humble-forward-command-controller \
    ros-humble-teleop-twist-keyboard \
    ros-humble-slam-toolbox \
    ros-humble-rmw-cyclonedds-cpp \
    x11-apps \
    gazebo \
    libgazebo-dev \
    unzip \
  && rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 2) Install and patch spawner.py for ros2_control
RUN mkdir -p /opt/ros/humble/lib/controller_manager && \
    curl -sL https://raw.githubusercontent.com/ros-controls/ros2_control_demos/humble/controller_manager/scripts/spawner.py \
      -o /opt/ros/humble/lib/controller_manager/spawner.py && \
    sed -i '1s|.*|#!/usr/bin/env python3|' /opt/ros/humble/lib/controller_manager/spawner.py && \
    chmod +x /opt/ros/humble/lib/controller_manager/spawner.py

# 3) Build gazebo_plugins into a separate workspace (/opt/gazebo_plugins_ws)
RUN mkdir -p /opt/gazebo_plugins_ws/src && \
    git clone -b ros2 https://github.com/ros-simulation/gazebo_ros_pkgs.git /opt/gazebo_plugins_ws/src/gazebo_ros_pkgs && \
    mv /opt/gazebo_plugins_ws/src/gazebo_ros_pkgs/gazebo_plugins /opt/gazebo_plugins_ws/src/ && \
    rm -rf /opt/gazebo_plugins_ws/src/gazebo_ros_pkgs && \
    . /opt/ros/humble/setup.sh && \
    cd /opt/gazebo_plugins_ws && \
    colcon build \
      --packages-select gazebo_plugins \
      --cmake-args -DBUILD_TESTING=OFF \
      --parallel-workers 1 --executor sequential && \
    echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/opt/gazebo_plugins_ws/install/gazebo_plugins/lib" >> /root/.bashrc && \
    echo "export GAZEBO_PLUGIN_PATH=\$GAZEBO_PLUGIN_PATH:/opt/gazebo_plugins_ws/install/gazebo_plugins/lib" >> /root/.bashrc

# 4) (Optional) Clone ros2_control_demos for reference
# RUN git clone -b humble https://github.com/ros-controls/ros2_control_demos.git /opt/ros2_control_demos

# 5) Install Gazebo models
RUN mkdir -p /usr/share/gazebo/models && \
    cd /usr/share/gazebo/models && \
    wget -q https://github.com/osrf/gazebo_models/archive/master.zip && \
    unzip master.zip && \
    mv gazebo_models-master/* . && \
    rm -rf gazebo_models-master master.zip && \
    chmod -R 755 /usr/share/gazebo/models

# 6) Install VNC + minimal window manager
RUN apt-get update && apt-get install -y \
    x11vnc \
    xvfb \
    fluxbox \
    xterm && \
    rm -rf /var/lib/apt/lists/*

# 7) Copy startup script and set permissions
COPY start_vnc.sh /start_vnc.sh
RUN chmod +x /start_vnc.sh

# 8) Set working directory and expose port
WORKDIR /root
EXPOSE 5900

# 9) Default entrypoint: start VNC
CMD ["/start_vnc.sh"]

# NOTE: Mount your local jackal_ws at runtime with:
# docker run -it --platform linux/amd64 -p 5900:5900 \
#   -v $(pwd)/jackal_ws:/root/jackal_ws jackal_vnc