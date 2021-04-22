# HDT Adroit Listener
The ```adroit_listener_node``` in this package is used to test the ```ros1_bridge``` on the HDT Adroit 6DOF A24 Pincer Manipulator Arm. 

## Example Usage
Although the source code for the ```hdt_6dof_a24_pincer``` cannot be provided, this step-by-step will explain how to use the ```ros1_bridge``` with the package.
1. Open a terminal and source for ROS noetic. Navigate to the catkin workspace with the ```hdt_6dof_a24_pincer``` source code and build the space.
```
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws/
catkin_make
```
2. Open a terminal and source for ROS foxy. Navigate to the colcon workspace you would like to work in. Clone the ```ros1_bridge``` source code in the ```src``` directory. The ```src``` directory should now have both the code ```ros1_bridge``` and the code within this repository. Build the workspace.
```
source /opt/ros/foxy/setup.bash
cd ~/colcon_ws/
git clone -b foxy https://github.com/ros2/ros1_bridge
colcon build
```
3. We are now ready to implement the bridge. Open a terminal, source the space for ROS noetic and start ```roscore```.
```
# Shell A (ROS1):
source /opt/ros/noetic/setup.bash
roscore
```
4. Open a terminal and source for ROS noetic and foxy. Run the ```ros1_bridge```.
```
# Shell B (ROS1 and ROS2):
source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash
ros2 run ros1_bridge dynamic_bridge
```
5. Open a new terminal and navigate to the catkin workspace. Source the workspace and launch the ```hdt_arm_bringup```.
```
# Shell C (ROS1):
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws/
source devel/setup.bash
roslaunch hdt_6dof_a24_pincer_bringup hdt_arm_bringup_1.launch
```
6. Open a new terminal and source for ROS2. Navigate to the colcon workspace and source the local setup. Run the ```adroit_listener_node```. 
```
# Shell D (ROS2):
source /opt/ros/foxy/setup.bash
cd ~/colcon_ws/
. install/local_setup.bash
ros2 run adroit_listener adroit_listener_node
```
7. If the xbox controller is not connected to the manipulator arm, then the following message will be displayed in the terminal running the ```adroit_listener_node```:
```
The xbox controller is not connected!
```
If the xbox controller connected, then the following message will be displayed:
```
The xbox controller is connected!
```