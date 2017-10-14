#!/usr/bin/env python
from rospy import Subscriber, Publisher
import rospy
import sys
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

rospy.init_node("OdomNode")
limit = float(sys.argv[1])
vel = float(sys.argv[2])
pub = Publisher('/diff_driver/command', Twist, queue_size=50)



def main():
  global v
  global dir1
  v=vel
  dir1=1
  rate = rospy.Rate(10);
  Subscriber('/diff_driver/odometry', Odometry, odomCallBack)
  while not rospy.is_shutdown():
	rate.sleep()

def odomCallBack(msg):
    global v
    global dir1
    print msg.pose.pose.position.x
    if msg.pose.pose.position.x > limit and dir1==1:
      v=-vel
      dir1=-1
    elif msg.pose.pose.position.x < -limit and dir1==-1:
      v=vel
      dir1=1
    
    send = Twist()
    send.linear.x = v
    pub.publish(send)

if __name__ == '__main__':
    main()

