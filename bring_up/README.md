# SOSVR2016 - bring_up
A package for spawning a robot ([pioneer3at](https://github.com/nkoenig/pioneer3at_demo)) into [Gazebo](http://gazebosim.org) with [ROS](http://www.ros.org).
### Usage
We assume you've installed ROS and Gazebo and configured it correctly. Then:
1. Open a new terminal. Then:
```
roscore
```
2. Open a new terimal. Then:

```
rosrun gazebo_ros gzserver
``` 
3. Open a new terimal. Then:

```
rosrun gazebo_ros gzclient
``` 
4. Clone this repository in your ROS workspace. Then *make* this package with `catkin_make` and *source* it.
5. In a new terminal:
```
roslaunch bring_up bring_up.launch
```
It should spwan a model into your gazebo client. :)
