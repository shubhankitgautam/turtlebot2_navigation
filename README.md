# turtlebot2_navigation
ROS2 Navigation with TurtleBot3 – SLAM, Nav2
TurtleBot3 Navigation Project
![ROS2](https://img.shields.io/badge/ROS2-Humble-blue) ![Gazebo](https://img.shields.io/badge/Gazebo-11-orange)

## Overview
This project demonstrates autonomous navigation using the TurtleBot3 Waffle in custom Gazebo worlds with ROS2. The robot uses **SLAM (Cartographer)** for mapping and **Nav2** for navigation. Wheel friction and damping are configured to reduce slipping and improve odometry accuracy.

---

## Features
- Custom Gazebo worlds for navigation experiments  
- SLAM-based mapping with Cartographer  
- Autonomous navigation with Nav2  
- Teleoperation using keyboard  
- Configured wheel friction and damping to prevent slipping  
- Realistic sensor simulation (LiDAR, IMU, camera)

---

## Requirements
- **OS:** Ubuntu 22.04  
- **ROS2:** Humble Hawksbill  
- **Gazebo:** 11  
- **TurtleBot3 Packages:** `turtlebot3_description`, `turtlebot3_gazebo`, `turtlebot3_navigation2`  
- Python 3, `colcon` build tool  

---

## Installation
Clone the repository into your ROS2 workspace:
cd ~/turtlebot3_ws/src
git clone <repository_url>

## Install dependencies:
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y

## Build the workspace:
cd ~/turtlebot3_ws
colcon build
source install/setup.bash

## Export TurtleBot3 model:
export TURTLEBOT3_MODEL=waffle

## Launching the Simulation
ros2 launch turtlebot3_gazebo turtlebot3_shubhankit_world.launch.py

##SLAM Mapping
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
ros2 run rviz2 rviz2
# Move the robot using teleop to generate a map:
ros2 run turtlebot3_teleop teleop_keyboard
# Save the generated map:
ros2 run nav2_map_server map_saver_cli -f maps/shubhankit_map

## Navigation
ros2 launch turtlebot3_navigation2 navigation_launch.py map:=maps/shubhankit_map.yaml
Use RViz2 to set navigation goals and monitor robot behavior.

## URDF & Physics Configuration
Wheel friction and damping are tuned to prevent slipping:
mu1=2.0, mu2=2.0
kp and kd values adjusted in <gazebo> wheel tags
Physics damping added in Gazebo link configuration
Sensors:
IMU (imu_link) for orientation and acceleration
LiDAR (base_scan) for obstacle detection
Camera (camera_rgb_frame) for vision-based tasks
Diff-drive plugin configured for proper odometry and control

## Custom World Notes
Friction values in your custom world may need adjustments. Very high friction can cause oscillations or wheel spinning.
maxVel in the diff-drive plugin should be low enough to prevent wheel overshoot.
Always source the workspace before launching commands:
source ~/turtlebot3_ws/install/setup.bash
Calibrate friction and damping depending on obstacles and surface type in custom worlds.

## Author
Shubhankit – Robotics Enthusiast / Mechanical Engineer

## References
[TurtleBot3 Official Documentation]([https://example.com](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/))
[ROS2 Navigation2 Guide](https://docs.nav2.org/)
[Cartographer SLAM](http://google-cartographer.readthedocs.io/en/latest/)


## License
This project is licensed under the MIT License. See the LICENSE file for details.
✅ This version includes:  
- Overview, features, requirements  
- Full installation and launch instructions  
- SLAM, navigation, URDF & physics notes  
- Custom world notes  
- Placeholders for screenshots and diagrams  
- References and license  



