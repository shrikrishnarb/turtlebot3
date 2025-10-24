# TurtleBot3 Supervised Navigation (ROS 2 Humble)

A safety supervisor for TurtleBot3 that intercepts `/cmd_vel` from Nav2, enforces slow/stop rules, and performs deterministic unblocking when obstacles persist in close proximity (3s timer → rotate → nudge forward/back → resume).

---

## Features
- **Gradual slowdown** near obstacles.
- **Blocked recovery**: Rotate + nudge routine after 3s.
- **Minimal dependencies**: Uses `/scan` and `/cmd_vel_raw`.
- **FSM**: PASS → SLOW → BLOCKED_WAIT → UNBLOCK_ROTATE → NUDGE_FORWARD → NUDGE_BACK → PASS.

---

## Prerequisites
- **OS / ROS 2**: Ubuntu 22.04, ROS 2 Humble.
- Install required packages:
  ```bash
  sudo apt update
  sudo apt install -y ros-humble-turtlebot3* \
    ros-humble-nav2-bringup ros-humble-slam-toolbox \
    ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins
   ```
- Set TurtleBot3 model:
   ```bash
   echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
   source ~/.bashrc
   ```
### Build
- From your ROS 2 workspace root:
```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## How to Run

### 1. Create a Map (SLAM + Map Saver)
Launch SLAM and RViz:
```bash
ros2 launch tb3_supervisor slam_and_map_save.launch.py use_sim_time:=True
```
Drive with keyboard:
```bash
ros2 run turtlebot3_teleop teleop_keyboard
```
Save map:
```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/turtlebot3_world
```

### 2. Bring Up Navigation with Supervisor
```bash
ros2 launch tb3_supervisor bringup_all.launch.py \
  use_sim_time:=True \
  map:=$HOME/maps/turtlebot3_world.yaml \
  x_pose:=-2.0 y_pose:=-0.5 z_pose:=0.0
```
- RViz opens → Click 2D Pose Estimate → Send Navigation Goal.

### (Optional) Send Goal via CLI
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: map}, pose: {position: {x: 1.5, y: 0.5, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}}"
```
 
 ---

 ## License
Apache-2.0. Please retain attribution and document changes if you redistribute.