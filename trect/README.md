# Trect Package
Launches the turtle simulator and contains a "start" service that will tell the robot what location to start moving in a rectangular trajectory. Package is built in ROS2.
# Example Usage

Open a terminal and source using the following command (NOTE: the command may differ depending on where ROS2 foxy installation is):
```
source /opt/ros/foxy/setup.bash
```

Navigate to the workspace and clone the git repository using the following command:
```
git clone https://github.com/sziselman/Brick-Building-Robot-Manipulation
```
Build the workspace:
```
colcon build
```
Open a new terminal and navigate to the work space. Source it:
```
source /opt/ros/foxy/setup.bash
```
```
. install/local_setup.bash
```
You can now launch the .py file:
```
ros2 launch trect trect_launch.py
```
In order to start the service, open a new terminal, source it and enter the following command:
```
source /opt/ros/foxy/setup.bash
```
```
ros2 service call /start trect/srv/Start "{x: 1, y: 2, width: 3, height: 4}"
```