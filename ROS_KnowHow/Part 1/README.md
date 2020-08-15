# Intro to ROS with turtlesim

### Intro to ROS.pdf 
A collection of some slides that explain ROS, ROS installation and ROS terminology.

### Tutorial

The tutorial is using the [`turtlesim`](http://wiki.ros.org/turtlesim) package. You need to install the package using the following command.

```sudo apt-get install ros-$(rosversion -d)-turtlesim``` 

You can start the tutorial with the following `launch` command.

```roslaunch tutle_zoom my_turtle.launch```

#### /src

##### my_keyboard_control.py

A publisher example that takes keyboard command and publishes `geometry_msgs/Twist` messages to corresponding topics to control turtles in the simulator.

##### my_sub.py

A basic subscriber script that listens the `turtlesim/Pose` message from `/turtle1/pose` topic and `geometry_msgs/Twist` from `/turtle1/cmd_vel` topic.