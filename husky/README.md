husky
=====

Common ROS packages for the Clearpath Husky, useable for both simulation and
real robot operation.

 - husky_control : Control configuration
 - husky_description : Robot description (URDF)
 - husky_msgs : Message definitions
 - husky_navigation : Navigation configurations and demos
 - husky_ur5_moveit_config : MoveIt configuration and demos

For Husky instructions and tutorials, please see [Robots/Husky](http://wiki.ros.org/Robots/Husky).

To create a custom Husky description or simulation, please fork [husky_customization](https://github.com/husky/husky_customization).

##Usage

We assume you've installed ROS and Gazebo and configured it correctly. Then:
1. Clone this repository somewhere and navigate there. Then open a new terminal there and run this commands:
```
$ catkin_make
$ source devel/setup.zsh
```
2. Then for launching the robot run:
```
$ roslaunch husky_gazebo husky_playpen.launch
```
3. For viewing the robot in RVIZ, run (in a new terminal):
```
$ roslaunch husky_viz view_robot.launch
```
4. For moving the robot and controling it, run (in a new terminal):
```
$ roslaunch husky_navigation move_base.launch 
```
5. For controlling robot's movements with a gamepad, run (in a new terminal):
```
$ roslaunch husky_control teleop.launch 
```
6. Finally, for building map, run (in a new terminal):
```
$ roslaunch husky_navigation move_base_mapless_demo.launch
```
