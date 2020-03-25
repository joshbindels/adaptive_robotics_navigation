#!/bin/bash

if [ $1 == "g" ] ;
then
	roslaunch turtlebot3_gazebo turtlebot3_maze.launch
elif [ $1 == "r" ] ;
then
	echo "start rviz";
	roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:\=gmapping
else
	echo "invalid";
fi
