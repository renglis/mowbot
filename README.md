# mowbot
```
docker buildx build --platform linux/amd64 -t jackal_sim .
```
```
docker run -it --rm \
  --platform linux/amd64 \
  --net=host \
  -e DISPLAY=host.docker.internal:0 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  jackal_sim
```
```
cd ~/jackal_ws
source /opt/ros/humble/setup.bash
sudo chown -R sim:sim ~/jackal_ws
```
password: sim
```
colcon build
source install/setup.bash
```
```
ros2 launch jackal_gazebo jackal_world.launch.py
```
