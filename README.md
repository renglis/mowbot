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
        └── velodyne_description/  # or from a stable fork
```

## 3. Build the Docker Image

From the same directory as your `Dockerfile` + `start_vnc.sh`:

```bash
docker buildx build --platform linux/amd64 -t jackal_vnc .
```

This installs ROS 2 Humble + Jackal dependencies + VNC packages + everything else needed.

## 4. Run the Container with VNC

```bash
docker run -it --rm \
  --platform linux/amd64 \
  -p 5900:5900 \
  jackal_vnc
```

- **Maps port 5900** so your Mac can VNC in at `127.0.0.1:5900`.
- Starts a **Fluxbox** window manager on **Xvfb** display `:1`.
- Launches **x11vnc** on port **5900**.
- Lands you in a **container shell** as well.

## 5. Connect via RealVNC

1. Open **RealVNC Viewer** (or any VNC client) on your Mac.
2. Connect to `127.0.0.1:5900` (or `localhost:5900`).

You’ll see a **blank Fluxbox desktop** – a grey screen. Right-click for the Fluxbox menu.

## 6. Open a Terminal **Inside the Same Container Shell**

If you’d like to open a graphical terminal window in Fluxbox:

1. **In the container shell** you already got from Step 4, run:
   ```bash
   export DISPLAY=:1
   xterm &
   ```
   This starts `xterm` in the VNC session on display `:1`, so you see it in Fluxbox.

2. Alternatively, you can add `xterm &` in your `start_vnc.sh` so it opens automatically when the container starts.

Now you can type commands in that **xterm** window within VNC.

## 7. Build and Launch Jackal

Inside either the xterm window in VNC **or** the same container shell (both ways are valid):

```bash
cd ~/jackal_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 launch jackal_gazebo jackal_world.launch.py
```

1. `colcon build` will create `install/` for your workspace.
2. Then source it (`source install/setup.bash`).
3. Finally, launch your simulation. Gazebo’s 3D window appears inside VNC.

## 8. Usage Tips

1. **Close/Stop**: Press `Ctrl+C` in the Docker shell or `docker stop <container_id>` from another Mac terminal.
2. **Performance**: Uses software rendering; slower than native GPU, but enough for typical robotics dev.
3. **Security**: By default, `x11vnc` is running **no password**. For local dev, that’s often fine. For broader networks, set a password via `x11vnc -usepw`.
4. **Resolution**: Tweak `Xvfb :1 -screen 0 1280x800x24` in `start_vnc.sh` if you need bigger resolution.

---

## Example `start_vnc.sh`

```bash
#!/usr/bin/env bash
# start_vnc.sh: spin up an X server + minimal WM + x11vnc in Docker

# 1) Virtual display at :1
Xvfb :1 -screen 0 1280x800x24 &
sleep 2

# 2) Start Fluxbox
fluxbox &
sleep 1

# 3) Start x11vnc on port 5900, no password
x11vnc -display :1 -forever -nopw -listen 0.0.0.0 -rfbport 5900 &

# Optionally open xterm automatically:
# DISPLAY=:1 xterm &

sleep 2
exec bash
```

That’s it! You have **VNC-based** Gazebo on macOS Docker, avoiding all OpenGL and XQuartz headaches. Happy simming!
