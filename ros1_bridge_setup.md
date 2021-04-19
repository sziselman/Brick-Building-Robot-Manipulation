# Bridging ROS and ROS2
This file documents how to bridge ROS and ROS2. 

## Create the ROS workspace
1. Navigate to the folder in which you'd like your workspace to be held and create a catkin workspace for any ROS packages.
```
mkdir -p catkin_ws/src
cd ws/src
```
Place ROS packages in the ```src``` folder. Source the workspace and build.
```
. /opt/ros/noetic/setup.bash
catkin_make
```

## Create the ROS2 workspace
2. Navigate to the folder in which you'd like your workspace to be held and create a colcon workspace for any ROS2 packages.
```
mkdir -p colcon_ws/src
cd colcon_ws/src
```
Place ROS2 packages in the ```src``` folder. Source the workspace and build.
```
. /opt/ros/foxy/setup.bash
colcon build
```

## Create the ROS1 Bridge workspace
3. Navigate to the folder in which you'd like your workspace to be held and create a workspace for the ROS1 to ROS2 bridge.
```
mkdir -p bridge_ws/src
cd bridge_ws/src
```
Clone the ROS1 bridge package in the source space.
```
git clone https://github.com/ros2/ros1_bridge
```
