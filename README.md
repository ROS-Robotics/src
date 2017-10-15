#    基于ROS系统的通用两轮底盘开发包
（Google cartographer 建图，导航，与底盘驱动接口的一体化程序包）

注意：在下载本程序包之前需要事前安装好全套的 ROS系统，如何安装ROS 系统可以参照下面的链接进行安装

https://github.com/ROS-Robotics/Webot-ROS-Setup-Docment/wiki/WeBot--软件安装（ROS系统及软件安装）


通用两轮底盘硬件开发平台介绍

https://github.com/ROS-Robotics/Webot-ROS-Setup-Docment/wiki/Webot-通用轮式机器人开发平台说明详解

在安装完成ROS 系统后便可下载安装本程序包了。

************** Download and bulid files **************************

    $ cd ~/
    $ mkdir webot
    $ cd webot

    $ sudo git clone https://github.com/ROS-Robotics/src.git

    $ sudo chmod -R 777 ./src/

    $ catkin_make_isolated --install --use-ninja

***********************************
        Run a demo  
***********************************        

$ source ~/webot/install_isolated/setup.bash

$ roslaunch webot webot_SLAM_test.launch
