# Jackal ROS 2 Humble Docker + VNC on macOS

## 1. Prerequisites

- **Docker Desktop** for macOS  
  - On Apple Silicon (M1/M2), remember to use `--platform linux/amd64` when building and running.
- **RealVNC Viewer** (or any other VNC client)

## 2. File Layout

You might have a structure like:

```
mowbot/
├── Dockerfile       # The Dockerfile (including VNC setup)
├── start_vnc.sh     # Script that launches Xvfb, fluxbox, x11vnc
└── jackal_ws/
    └── src/
        ├── jackal/
        ├── jackal_simulator/
        ├── velodyne/
        └── velodyne_description/
```

## 3. Build the Docker Image

From the same directory as your `Dockerfile` + `start_vnc.sh`:

```bash
docker buildx build --platform linux/amd64 -t jackal_vnc .
```

This installs ROS 2 Humble + Jackal dependencies + VNC packages + everything else needed.

## 4. Run the Container with VNC

```bash
docker run -it \
  --platform linux/amd64 \
  -p 5900:5900 \
  -v $(pwd)/jackal_ws:/root/jackal_ws \
  jackal_vnc
```

- **Maps port 5900** so your Mac can VNC in at `127.0.0.1:5900`.
- Starts a **Fluxbox** window manager on **Xvfb** display `:1`.
- Launches **x11vnc** on port **5900**.
- Lands you in a **container shell** as well.
- Persists local changes into docker container

## 5. Connect via RealVNC

1. Open **RealVNC Viewer** (or any VNC client) on your Mac.
2. Connect to `127.0.0.1:5900` (or `localhost:5900`).

You’ll see a **blank Fluxbox desktop** – a grey screen.

## 6. Inside the Same Container Shell

1. **In the container shell** you already got from Step 4, run:
   ```bash
   export DISPLAY=:1
   xterm &
   ```
   This starts `xterm` in the VNC session on display `:1`, so you see it in Fluxbox.

Now you can type commands in that **xterm** window within VNC.

## 7. Build and Launch Jackal

Inside either the xterm window in VNC **or** the same container shell (both ways are valid):

```bash
cd jackal_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
export JACKAL_LASER=1; export JACKAL_LASER_MODEL=ust10
ros2 launch jackal_gazebo jackal_world.launch.py config:=front_laser use_sim_time:=true
```

1. `colcon build` will create `install/` for your workspace.
2. Then source it (`source install/setup.bash`).
3. Finally, launch your simulation. Gazebo’s 3D window appears inside VNC.

separate term:
```bash
source /opt/ros/humble/setup.bash
source ~/jackal_ws/install/setup.bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true slam_params_file:=/root/jackal_ws/src/jackal/jackal_navigation/config/slam.yaml
```
separate term:
```bash
source /opt/ros/humble/setup.bash
source ~/jackal_ws/install/setup.bash
ros2 launch jackal_navigation nav2.launch.py use_sim_time:=true
```
separate term:
```bash
source /opt/ros/humble/setup.bash
source ~/jackal_ws/install/setup.bash
ros2 run explore_lite explore \
  --ros-args -p costmap_topic:=/global_costmap/costmap \
             -p use_sim_time:=true
```

## 8. Usage Tips

1. **Close/Stop**: Press `Ctrl+C` in the Docker shell or `docker stop <container_id>` from another Mac terminal.
2. **Performance**: Uses software rendering; slower than native GPU, but enough for typical robotics dev.
3. **Security**: By default, `x11vnc` is running **no password**. For local dev, that’s often fine. For broader networks, set a password via `x11vnc -usepw`.
4. **Resolution**: Tweak `Xvfb :1 -screen 0 1280x800x24` in `start_vnc.sh` if you need bigger resolution.


-----


# 🚀 Running Jackal Controllers & Moving in Gazebo (ROS 2 + Docker)

## 1. Connect to Docker on a separate terminal than the one already running the jackal sim

Enter container:

```bash
docker exec -it <container_id_or_name> bash
```

## 2. Source ROS + Workspace

Inside Docker:

```bash
source /opt/ros/humble/setup.bash
source ~/jackal_ws/install/setup.bash
```

## 3. Start Controllers

Run:

```bash
ros2 run controller_manager spawner joint_state_broadcaster --controller-manager /controller_manager
ros2 run controller_manager spawner jackal_velocity_controller --controller-manager /controller_manager
ros2 run controller_manager spawner blade_velocity_controller --controller-manager /controller_manager

```

Verify:

```bash
ros2 control list_controllers
```

Expected:
```
joint_state_broadcaster [active]
jackal_velocity_controller [active]
```

## 4. EXAMPLE -- Move Jackal in Gazebo


```bash
sudo apt install ros-humble-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Use `Ctrl+C` to stop.

