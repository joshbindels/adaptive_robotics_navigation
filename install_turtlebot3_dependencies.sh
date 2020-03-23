#!/bin/bash

if [[ -d ~/catkin_ws/ ]]
then
    echo "catkin_ws already exists.."
else
    echo "Creating catkin_ws.."
    mkdir -p ~/catkin_ws/src
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
fi

echo "Downloading TurtleBot3 simulator.."
cd ~/catkin_ws/src
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
echo "Building TurtleBot3 simulator.."
cd ..
catkin_make
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc

echo "Downloading TurtleBot3 simulation files..."
source ~/.bashrc
cd ~/catkin_ws/src
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ..
catkin_make


echo "TurtleBot3 simulator installed"

