#!/usr/bin/env python

import roslib; 
import rospy
from sensor_msgs.msg import Joy 
from geometry_msgs.msg import Twist
from ric_robot.msg import ric_pan_tilt

def joy_callback(data):
    global msg
    global msg_pt
    global max_vel
    global max_rot
    global msg_pt
    global max_pan
    global max_tilt
    global pan_vel
    global tilt_vel
    global pan_dir
    global tilt_dir
    global scroll_btn
    global current_robot
    global num_robots
    global joy_for
    global joy_rot
    global joy_pan
    global joy_tilt
    global joy_scroll_btn
    global joy_slow_btn
    global slow
    global slow_btn
    global joy_center_btn
    global center_btn
    global center
    msg.linear.x=data.axes[joy_for]*max_vel*slow
    msg.angular.z=data.axes[joy_rot]*max_rot*slow
    if (data.axes[joy_pan]>0.5 and msg_pt.pan_angle<max_pan):
      pan_dir=1
    elif (data.axes[joy_pan]<-0.5 and msg_pt.pan_angle>-max_pan):
      pan_dir=-1
    else:
      pan_dir=0
    if (data.axes[joy_tilt]>0.5 and msg_pt.tilt_angle<max_tilt):
      tilt_dir=1
    elif (data.axes[joy_tilt]<-0.5 and msg_pt.tilt_angle>-max_tilt):
      tilt_dir=-1
    else:
      tilt_dir=0
    if (data.buttons[joy_scroll_btn]==1):
       if (scroll_btn==0):
          current_robot=current_robot+1
          scroll_btn=1
          if (current_robot>num_robots):
             current_robot=1
          rospy.loginfo("Controlling robot with ID %d",current_robot)
    if (data.buttons[joy_scroll_btn]==0):
       scroll_btn=0
    if (data.buttons[joy_slow_btn]==1):
       if (slow_btn==0):
          if (slow==0.5):
	     slow=1.0
	  else:
	     slow=0.5
          slow_btn=1
          rospy.loginfo("Max velocity is %f m/s",max_vel*slow)
    if (data.buttons[joy_slow_btn]==0):
       slow_btn=0
    if (data.buttons[joy_center_btn]==1):
       if (center_btn==0):
          msg_pt.pan_angle=0
          msg_pt.tilt_angle=0
	  center=0
          center_btn=1
          rospy.loginfo("Centering pan tilt system")
    if (data.buttons[joy_center_btn]==0):
       center_btn=0


def joy_cam():
    global pub 
    global pub_pt 
    global msg
    global msg_pt
    global max_pan
    global max_tilt
    global pan_vel
    global tilt_vel
    global pan_dir
    global tilt_dir
    global max_vel
    global max_rot
    global num_robots
    global scroll_btn
    global current_robot
    global joy_for
    global joy_rot
    global joy_pan
    global joy_tilt
    global joy_scroll_btn
    global joy_slow_btn
    global slow
    global slow_btn
    global joy_center_btn
    global center_btn
    global center
    joy_for = rospy.get_param("joy_for")
    joy_rot = rospy.get_param("joy_rot")
    joy_pan = rospy.get_param("joy_pan")
    joy_tilt = rospy.get_param("joy_tilt")
    joy_scroll_btn = rospy.get_param("joy_scroll_btn")
    joy_slow_btn = rospy.get_param("joy_slow_btn")
    joy_center_btn = rospy.get_param("joy_center_btn")
    max_vel = rospy.get_param("max_vel")
    max_rot = rospy.get_param("max_rot")
    max_pan = rospy.get_param("max_pan")
    max_tilt = rospy.get_param("max_tilt")
    num_robots = rospy.get_param("robots_n")
    robot_type = rospy.get_param("robot_type")
    center=10
    slow=1
    slow_btn=0
    center_btn=0
    pan_dir=0
    tilt_dir=0
    pan_vel=0.02
    tilt_vel=0.02
    pub=range(num_robots)
    pub_pt=range(num_robots)
    scroll_btn=0
    current_robot=1
    msg = Twist()
    msg.linear.x=0
    msg.linear.y=0
    msg.linear.z=0
    msg.angular.x=0
    msg.angular.y=0
    msg.angular.z=0
    msg_pt = ric_pan_tilt()
    msg_pt.pan_angle=0
    msg_pt.tilt_angle=0
    rospy.init_node('joy_cam', anonymous=True)
    rospy.Subscriber('joy', Joy, joy_callback)
    rospy.loginfo("Controlling robot with ID %d",current_robot)
    for i in range(1, num_robots+1):
      robot_name=robot_type+"_"+str(i)+"/cmd_vel"
      #rospy.loginfo(robot_name) 
      pub[i-1] = rospy.Publisher(robot_name, Twist)
      pub[i-1].publish(msg)
      rospy.sleep(0.1)
      robot_name_pt=robot_type+"_"+str(i)+"/pan_tilt"
      pub_pt[i-1] = rospy.Publisher(robot_name_pt, ric_pan_tilt)
      pub_pt[i-1].publish(msg_pt)
      rospy.sleep(0.1)
    rospy.loginfo("Ready to control, Controlling robot with ID: %d",current_robot)
    rospy.loginfo("Max velocity is %f m/s",max_vel*slow)
    while not rospy.is_shutdown():
      pub[current_robot-1].publish(msg)
      msg_pt.pan_angle=msg_pt.pan_angle+pan_dir*pan_vel
      msg_pt.tilt_angle=msg_pt.tilt_angle+tilt_dir*tilt_vel
      if (msg_pt.pan_angle>max_pan):
        msg_pt.pan_angle=max_pan
      if (msg_pt.pan_angle<-max_pan):
        msg_pt.pan_angle=-max_pan
      if (msg_pt.tilt_angle>max_tilt):
        msg_pt.tilt_angle=max_tilt
      if (msg_pt.tilt_angle<-max_tilt):
        msg_pt.tilt_angle=-max_tilt
      if (pan_dir<>0 or tilt_dir<>0):
        pub_pt[current_robot-1].publish(msg_pt)
      #rospy.loginfo(current_robot)
      rospy.sleep(0.1)

if __name__ == '__main__':
    rospy.loginfo("Starting joy_cam.py node")
    joy_cam()

