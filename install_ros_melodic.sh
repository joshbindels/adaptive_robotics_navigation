echo "Intalling ros melodic.."

echo "Setting up sources list.."
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

echo "Setting up keys.."
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

echo "Updating.."
sudo apt update

echo "Installing ros.."
sudo apt install ros-melodic-desktop-full

echo "Setting up rosdep.."
sudo rosdep init
rosdep update

echo "Saving source to bashrc"
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "Finished installing ros melodic."
