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
source /opt/ros/noetic/setup.bash
catkin_make
```

## Create the ROS2 workspace
2. Navigate to the folder in which you'd like your workspace to be held and create a colcon workspace for any ROS2 packages.
```
mkdir -p colcon_ws/src
cd colcon_ws/src
```
Place ROS2 packages in the ```src``` folder. 

## Using the ROS1 to ROS2 bridge
3. Clone the ROS1 bridge package in the ```src``` directory of the ```colcon_ws```. Note: I cloned the foxy branch, as it matches my ROS release.
```
git clone -b foxy https://github.com/ros2/ros1_bridge
```
## Variable Mapping
4. This step may not be necessary, depending on how variables in your messages and services are named. ROS2 has stricter linting rules and CamelCase variables are no longer used. In this step, re-name variables that are usable in ROS, but otherwise not useable in ROS2.

5. Depending on if variables were re-named or not, create a .yaml file that linkes the new name to the old name.

## Build the ROS2 workspace
6. Source the ```colcon_ws``` and build.
```
source /opt/ros/foxy/setup.bash
colcon build
```

## Run the ROS1 to ROS2 bridge
7. Now we run the bridge!!!! Open a terminal and start a ROS1 ```roscore```
```
# Shell A (ROS1):
source /opt/ros/noetic/setup.bash
roscore
```

Open a new terminal and source the space for both ROS1 and ROS2. Run the bridge.
```
# Shell B (ROS2):
source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash
ros2 run ros1_bridge dynamic_bridge
```

## Start the ROS1 node
8. Now, we start the ROS1 node in a new terminal. In this example, I am using my ROS1 ```turtle_rect``` node from the following repository: https://github.com/sziselman/Shermbot 
```
# Shell C (ROS1):
source /opt/ros/noetic/setup.bash
rosrun trect turtle_rect
```

## Start the ROS2 node
9. Now, we start the ROS2 node in a new terminal. In this example, I am using the ROS2 ```turtlesim_node``` node from the foxy-devel branch of the following repository: https://github.com/ros/ros_tutorials .
```
# Shell D (ROS2):
source /opt/ros/foxy/setup.bash
ros2 run turtlesim turtlesim_node
```