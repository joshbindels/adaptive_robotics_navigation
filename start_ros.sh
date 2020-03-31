#!/bin/bash

if [ $1 == "g" ] ;
then
	roslaunch turtlebot3_gazebo turtlebot3_maze.launch x_pos:=-5.95 y_pos:=-4.44
elif [ $1 == "r" ] ;
then
	echo "start rviz";
	roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/home/josh/catkin_ws/src/map/mymap.yaml
else
	echo "invalid";
fi
