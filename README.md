# Jackal ROS 2 Humble Docker + VNC on macOS

## 1. Prerequisites

- **Docker Desktop** for macOS  
  - On Apple Silicon (M1/M2), remember to use `--platform linux/amd64` when building and running.
- **RealVNC Viewer** (or any other VNC client)

## 2. File Layout

```
mowbot/
├── Dockerfile
├── start_vnc.sh
└── jackal_ws/
    └── src/
        ├── jackal/
        ├── jackal_simulator/
        ├── velodyne/
        ├── velodyne_description/
        └── scripts/
            ├── pointcloud_to_range_node.py
            └── blade_control_node.py
```

## 3. Build the Docker Image

```bash
docker buildx build --platform linux/amd64 -t jackal_vnc .
```

## 4. Run the Container with VNC

```bash
docker run -it \
  --platform linux/amd64 \
  -p 5900:5900 \
  -v $(pwd)/jackal_ws:/root/jackal_ws \
  jackal_vnc
```

## 5. Connect via RealVNC

1. Open **RealVNC Viewer**.
2. Connect to `127.0.0.1:5900`.

## 6. Inside the Same Container Shell

```bash
export DISPLAY=:1
xterm &
```

## 7. Build and Launch Jackal

```bash
cd jackal_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
export JACKAL_LASER=1
export JACKAL_LASER_MODEL=ust10
ros2 launch jackal_gazebo jackal_world.launch.py config:=front_laser use_sim_time:=true
```

---

## 🚗 Autonomous Navigation with SLAM + LiDAR

To enable autonomous mapping and exploration using SLAM Toolbox and Nav2, open **three separate terminals** inside the Docker container:

### **Terminal 1: SLAM Toolbox**

```bash
source /opt/ros/humble/setup.bash
source ~/jackal_ws/install/setup.bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true \
  slam_params_file:=/root/jackal_ws/src/jackal/jackal_navigation/config/slam.yaml
```

### **Terminal 2: Nav2 Navigation Stack**

```bash
source /opt/ros/humble/setup.bash
source ~/jackal_ws/install/setup.bash
ros2 launch jackal_navigation nav2.launch.py use_sim_time:=true
```

### **Terminal 3: Explore Lite for Frontier Exploration**

```bash
source /opt/ros/humble/setup.bash
source ~/jackal_ws/install/setup.bash
ros2 run explore_lite explore \
  --ros-args -p costmap_topic:=/global_costmap/costmap \
             -p use_sim_time:=true
```

---

## 🌾 Launching the Mowing Mechanism

The following two nodes handle mowing logic based on ultrasonic range detection:

### **Terminal 4: Convert PointCloud2 to Range**

```bash
source /opt/ros/humble/setup.bash
source ~/jackal_ws/install/setup.bash
chmod +x jackal_ws/src/scripts/pointcloud_to_range_node.py
python3 jackal_ws/src/scripts/pointcloud_to_range_node.py
```

### **Terminal 5: Blade Activation Logic**

```bash
source /opt/ros/humble/setup.bash
source ~/jackal_ws/install/setup.bash
chmod +x jackal_ws/src/scripts/blade_control_node.py
python3 jackal_ws/src/scripts/blade_control_node.py
```

---

## 🔧 Running Jackal Controllers & Moving in Gazebo

### 1. Open a New Terminal and Attach to the Container

```bash
docker exec -it <container_id_or_name> bash
```

### 2. Source ROS 2 and Your Workspace

```bash
source /opt/ros/humble/setup.bash
source ~/jackal_ws/install/setup.bash
```

### 3. Spawn the Controllers

```bash
ros2 run controller_manager spawner joint_state_broadcaster --controller-manager /controller_manager
ros2 run controller_manager spawner jackal_velocity_controller --controller-manager /controller_manager
ros2 run controller_manager spawner blade_velocity_controller --controller-manager /controller_manager
```

### 4. Verify the Controllers

```bash
ros2 control list_controllers
```

Expected output:
```
joint_state_broadcaster [active]
jackal_velocity_controller [active]
blade_velocity_controller [active]
```

### 5. (Optional) Drive the Jackal Manually

```bash
sudo apt install ros-humble-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Use arrow keys to drive. Press `Ctrl+C` to stop.

---
