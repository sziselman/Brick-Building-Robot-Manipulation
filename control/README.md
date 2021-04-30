# Control
Package used for the controls of the HDT Adroit Manipulator Arm

__Example Usage:__
First, we want to launch the HDT Adroit Manipulator Arm software. It is important to note that the ```hdt_6dof_a24_pincer``` is needed to run the ```control``` package.
```
roslaunch hdt_6dof_a24_pincer_bringup hdt_arm_bringup_1.launch simulation:=true
```
Next, we want to launch the ```control``` package for the Adroit arm.
```
roslaunch control control.launch
```