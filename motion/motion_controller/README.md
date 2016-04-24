motion_controller
=============================

A ROS Indigo package implementing motion controller for robot.
Publishes on cmd_vel topic using twist linear.x speed and linear.z speed and geometry_msgs.mgs.

## Usage
[Install ROS](http://wiki.ros.org/ROS/Installation)

* $ cd [your indigo catkin src folder]
* $ git clone git@github.com:taherahmadi/SOSVR2016.git
* $ cd ..
* $ catkin_make
* $ roslaunch motion_controller motion_controller.launch
* Use w,s,d,a,l,k keys to move

