# Brick Building Robot Manipulation
Graduate-level independent project written by Sarah Ziselman utilizing the HDT Adroit A24 Manipulator Arm and MoveIt! C++ API to pick and place bricks to build a wall. Navigation will be done through collaborator using a Clear Path Jackal UGV robot.

## Introduction
This project explores motion planning and manipulation of the HDT Adroit Arm in MoveIt!'s C++ API. The goal is to integrate the Adroit Robot with the Jackal UGV robot to create a brick building robot. The manipulation portion of the project will be used to pick up bricks and stack them in a wall formation depending on the approximate amount of bricks identified. This application is both practical and fun to work with, as it can work to increase efficiency in building structures.

## Hardware
## Software
ROS Noetic needs to be installed to work with the software in this repository. See the following link: https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html

MoveIt! needs to also be built from source, so that the robot's motion planning can be planned, executed and simulated. See the following link: https://moveit.ros.org/install/source/

The HDT Adroit software must be installed in the project workspace. This software requires permission to acccess.

Lastly, clone this repository in the project workspace and run the following commands:
```
catkin_make
source devel/setup.bash
```

## Packages
## Implementation

## Notes for Integration with Brick Building Navigation software
Follow the instructions at the following repository to set up the Navigation software: https://github.com/nan0314/Brick-Building-Robot-Navigation. Make sure that the above steps above to set up the software have been followed and are in the same workspace.

## Summary
__This repository contains the following packages:__
* control - a package that is used for the controls of the HDT Adroit Manipulator Arm.