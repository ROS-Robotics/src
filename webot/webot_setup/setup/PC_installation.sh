#!/bin/bash

#Terminator
sudo apt-get -y install terminator
#END 

#ROS install
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
sudo apt-get update
sudo apt-get -y install ros-indigo-desktop-full
sudo rosdep init
rosdep update
sudo apt-get -y install python-rosinstall
source /opt/ros/indigo/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
cd ~/catkin_ws/src
git clone https://github.com/robotican/ric.git
cd ~/catkin_ws/src/ric/ric/setup
sudo ./setup.sh
cd ~/catkin_ws
catkin_make
sudo rosdep fix-permissions
sudo chown $USER:$USER ~/catkin_ws
exit
#END
