# Control
Package used for the controls of the HDT Adroit Manipulator Arm in the Brick Building Robot project. Package includes the node ```arm_control```, which uses MoveIt! software to plan and execute motions of the arm and pincer. Also contains a library called ```control_library```, which contains functions used in the ```arm_control``` node.

## Publishers & Subscribers
* pincer_pub, which publishes a std_msgs/Float64 message to the topic /hdt_arm/pincer_joint_position_controller/command

## Broadcasters & Listeners
* brick_listener, which listens for a transform that contains the location of the identified brick.

## Services & Clients
* ```rosservice call /stow_position``` : Moves the adroit arm to the stow position. Robot starts and finishes in stow position. Robot also moves to stow position once it has picked up a brick and the jackal drives to the designated location.
* ```rosservice call /pregrasp_position``` : Moves the adroit arm to the pre-grasp position, which is 0.10 m above the brick's center (z-direction) that it is going to grasp.
* ```rosservice call /grasp_position``` : Moves the adroit arm to the grasp position, which is 0.01 m below the brick's center (z-direction) that it is going to grasp.
* ```rosservice call /place_position``` : Moves the adroit arm to the designated place position of the brick.
* ```rosservice call /open_pincers``` : Opens the adroit pincers, so that it can prepare to grasp a brick.
* ```rosservice call /close_pincers``` : Closes the adroit pincers, so that it can either grasp a brick or close the pincers in resting position.

__Example Usage:__
First, make sure to source the ROS distro.
```
source /opt/ros/noetic/setup.bash
```
Next, we want to launch the ```control``` package for the Adroit arm.
```
roslaunch control control.launch
```
This will launch the MoveIt! simulator in RViz. In order to move the arm, call one of the services listed above.