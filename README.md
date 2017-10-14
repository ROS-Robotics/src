# src

**************Download and bulid files **************************

    $ cd ~/
    $ mkdir webot
    $ cd webot

    $ sudo git clone git@github.com:ROS-Robotics/src.git

    $ chmod -R 777 ./src/

    $ catkin_make_isolated --install --use-ninja

***********************************
        Run a demo  
***********************************        

$ source ~/webot/install_isolated/setup.bash

$ roslaunch webot webot_SLAM_test.launch
