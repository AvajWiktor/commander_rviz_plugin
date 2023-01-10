# Rviz Commander Plugin

## Installation ROS2 Foxy
### 1. Get started
```
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/AvajWiktor/commander_rviz_plugin.git
cd ..
source /opt/ros/foxy/setup.bash
colcon build
```
### 2. Rviz usage
```
cd catkin_ws
source install/setup.bash
rviz2
```
 - Load rviz config from config directory
![alt text](https://github.com/AvajWiktor/commander_rviz_plugin/tree/main/images/rviz_config.png?raw=true)
 - Use arrows and velocity scale slider to move Husky
![alt text](https://github.com/AvajWiktor/commander_rviz_plugin/tree/main/images/rviz_usage.png?raw=true)