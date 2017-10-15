#    基于ROS系统的通用两轮底盘开发包
（Google Cartographer 建图，导航，底盘驱动接口(Robot Drive-Kit 和上层ROS 系统通信）的全集成一体化程序包）

注意：在下载本程序包之前需要事前安装好全套的 ROS系统，如何安装ROS 系统可以参照下面的链接进行安装

https://github.com/ROS-Robotics/Webot-ROS-Setup-Docment/wiki/WeBot--软件安装（ROS系统及软件安装）


通用两轮底盘硬件开发平台介绍

https://github.com/ROS-Robotics/Webot-ROS-Setup-Docment/wiki/Webot-通用轮式机器人开发平台说明详解
Robot Drive-Kit 套件组成说明：https://github.com/ROS-Robotics/ros_drv_bridge/wiki/ROS_DRV_BRIDGE_README 


在安装完成ROS 系统后便可下载安装本程序包了。

************** Download and bulid files **************************

    $ cd ~/
    $ mkdir webot
    $ cd webot

    $ sudo git clone https://github.com/ROS-Robotics/src.git

    $ sudo chmod -R 777 ./src/

    $ catkin_make_isolated --install --use-ninja



编译完成后在运行程序之前请确认硬件各接口已经连接到位

1.Robot Drive-Kit(底盘驱动)的串口接到 ROS 主机端（Robot Drive-Kit 套件组成说明：https://github.com/ROS-Robotics/ros_drv_bridge/wiki/ROS_DRV_BRIDGE_README ）

2.3D摄像头使用的是INTEL realsense R200 连接到 ROS 主机 USB3.0接口上。

***********************************
        Run a demo  
***********************************        

$ source ~/webot/install_isolated/setup.bash

$ roslaunch webot webot_SLAM_test.launch

