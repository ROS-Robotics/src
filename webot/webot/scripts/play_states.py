#!/usr/bin/env python
###########################
# play_states.py
# This program play all recorded states of the komodo_arm motors from a file. 
###########################
import roslib; roslib.load_manifest('ric_robot')
import rospy
from std_msgs.msg import Float64

def play_states():
    rospy.init_node('play_states', anonymous=True)
    # open file:
    f = open('states_log.txt', 'r')
    #read from file
    data = [map(float, line.split()) for line in f] 
    l=len(data)
    pub_base_rotation=rospy.Publisher('/komodo_1/base_rotation_controller/command', Float64)
    pub_shoulder = rospy.Publisher('/komodo_1/shoulder_controller/command', Float64)
    pub_elbow1 = rospy.Publisher('/komodo_1/elbow1_controller/command', Float64)
    pub_elbow2 = rospy.Publisher('/komodo_1/elbow2_controller/command', Float64)
    pub_wrist = rospy.Publisher('/komodo_1/wrist_controller/command', Float64)
    pub_left_finger = rospy.Publisher('/komodo_1/left_finger_controller/command', Float64)
    pub_right_finger = rospy.Publisher('/komodo_1/right_finger_controller/command', Float64)
    for i in range(0,l):
      if i==5:
        rospy.sleep(0.1)
      br = Float64(data[i][0])
      sh = Float64(data[i][1])
      e1 = Float64(data[i][2])
      e2 = Float64(data[i][3])
      wr = Float64(data[i][4])
      lf = Float64(data[i][5])
      rf = Float64(data[i][6])
      rospy.loginfo('base_rotation_'+str(br))
      rospy.loginfo('shoulder_'+str(sh))
      rospy.loginfo('elbow1_'+str(e1))
      rospy.loginfo('elbow2_'+str(e2))
      rospy.loginfo('wrist_'+str(wr))
      rospy.loginfo('left_finger_'+str(lf))
      rospy.loginfo('right_finger_'+str(rf))
      pub_base_rotation.publish(br)
      pub_shoulder.publish(sh)
      pub_elbow1.publish(e1)
      pub_elbow2.publish(e2)
      pub_wrist.publish(wr)
      pub_left_finger.publish(lf)
      pub_right_finger.publish(rf)
      rospy.sleep(3.0)
    
if __name__ == '__main__':
    #rospy.loginfo("Starting play_converter.py")
    play_states()

