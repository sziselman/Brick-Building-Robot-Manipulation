# Bridging ROS and ROS2
This file documents how to bridge ROS and ROS2. This allows for ROS1 nodes to be implemented in ROS2 nodes.

## Create the ROS workspace
1. Navigate to the folder in which you'd like your workspace to be held and create a catkin workspace for any ROS packages.
```
mkdir -p catkin_ws/src
cd catkin_ws/src
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

3. Clone the ROS1 bridge package in the ```src``` directory of the ```colcon_ws```. Note: I cloned the foxy branch, as it matches my ROS release.
```
git clone -b foxy https://github.com/ros2/ros1_bridge
```
## Variable Mapping
4. This step may not be necessary, depending on how variables in your messages and services are named. ROS2 has stricter linting rules and CamelCase variables are no longer used. In this step, re-name variables that are usable in ROS, but otherwise not useable in ROS2.

5. Depending on if variables were re-named or not, create a .yaml file that links the new name to the old name.

## Build the ROS2 workspace
6. Source the ```colcon_ws``` and build.
```
source /opt/ros/foxy/setup.bash
colcon build
```

## Implementing the Bridge
7. Now we run the bridge!!!! Open a terminal and start a ROS1 ```roscore```
```
# Shell A (ROS1):
source /opt/ros/noetic/setup.bash
roscore
```

Open a new terminal and source the space for both ROS1 and ROS2. Since we are using ```dynamic_bridge``` from our source file, navigate to the ```colcon_ws``` and source the local setup. It is important to note that you should not source the local setup in the same shell that ```colcon_ws``` was built in. Run the bridge.
```
# Shell B (ROS2):
source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash
cd ~/colcon_ws/
. install/local_setup.bash
ros2 run ros1_bridge dynamic_bridge
```

## Start the ROS1 node
8. Now, we start the ROS1 node in a new terminal. In this example, I am using my ROS1 ```turtle_rect``` node from the following repository: https://github.com/sziselman/Shermbot. Navigate to the ```catkin_ws``` and source the local setup. We also need to set the parameters for the ```turtle_rect``` node. Run the noe.
```
# Shell C (ROS1):
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws/
source devel/setup.bash
rosparam set max_xdot 2.0
rosparam set max_wdot 1.0
rosparam set frequency 100
rosrun trect turtle_rect
```

## Start the ROS2 node
9. Now, we start the ROS2 node in a new terminal. In this example, I am using the ROS2 ```turtlesim_node``` node from the foxy-devel branch of the following repository: https://github.com/ros/ros_tutorials. Since we are using the ```turtlesim_node``` from our source file, navigate to the ```colcon_ws``` and source the local setup. Run the node.
```
# Shell D (ROS2):
source /opt/ros/foxy/setup.bash
cd ~/colcon_ws/
. install/local_setup.bash
ros2 run turtlesim turtlesim_node
```
# Test the bridge
10. We test the bridge by calling a service in the ROS1 node, which will be implemented in the ROS2 node. I wrote a ```/start``` service that will have the turtle start at a designated (x,y) position and then move in a rectangular trajectory based on a designated width and height. Open a new terminal. Source the workspace. Call the service.
```
# Shell E (ROS1):
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws/
source devel/setup.bash
rosservice call /start "x:1, y:2, width:3, height:4"
```
You will now see that the ROS2 ```turtlesim_node``` will run the ```/start``` service.