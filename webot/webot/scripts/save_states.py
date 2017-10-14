#!/usr/bin/env python
###########################
# save_states.py
# This program records the states of the komodo_arm motors into a file. 
###########################
import roslib; roslib.load_manifest('ric_robot')
import rospy
from dynamixel_msgs.msg import JointState as dxl_JointState
from sensor_msgs.msg import JointState

def br_callback(data):       
    global msg
    msg[0]=data.current_pos

def sh_callback(data):       
    global msg
    msg[1]=data.current_pos

def e1_callback(data):       
    global msg
    msg[2]=data.current_pos

def e2_callback(data):       
    global msg
    msg[3]=data.current_pos

def wr_callback(data):       
    global msg
    msg[4]=data.current_pos

def lf_callback(data):       
    global msg
    msg[5]=data.current_pos

def rf_callback(data):       
    global msg
    msg[6]=data.current_pos

def save_states():
    rospy.init_node('save_states', anonymous=True)
    # open file:
    f = open('states_log.txt', 'w')
    #write to file
    #f.write('base_rotation, shoulder, elbow1, elbow2, wrist, left_finger, right_finger\n')
    global msg
    msg = [0,0,0,0,0,0,0]
    rospy.Subscriber("/komodo_1/base_rotation_controller/state", dxl_JointState, br_callback)
    rospy.Subscriber("/komodo_1/shoulder_controller/state", dxl_JointState, sh_callback)
    rospy.Subscriber("/komodo_1/elbow1_controller/state", dxl_JointState, e1_callback)
    rospy.Subscriber("/komodo_1/elbow2_controller/state", dxl_JointState, e2_callback)
    rospy.Subscriber("/komodo_1/left_finger_controller/state", dxl_JointState, lf_callback)
    rospy.Subscriber("/komodo_1/right_finger_controller/state", dxl_JointState, rf_callback)
    rospy.Subscriber("/komodo_1/wrist_controller/state", dxl_JointState, wr_callback)
    i=1
    while not rospy.is_shutdown():
        br=msg[0]
        sh=msg[1]
        e1=msg[2]
        e2=msg[3]
        wr=msg[4]
        lf=msg[5]
        rf=msg[6]
        if i>1:
          print 'iteration=', i
          p= raw_input("Press ENTER to continue or q to quit")
          print "input=", p
          if p=='q': 
             return
          print 'msg=', msg
          f.write(str(br)+" "+str(sh)+" "+str(e1)+" "+str(e2)+" "+str(wr)+" "+str(lf)+" "+str(rf))
          f.write('\n')
          rospy.sleep(0.1)
        i=i+1
        rospy.sleep(0.1)
	    #rospy.loginfo(msg)

if __name__ == '__main__':
    rospy.loginfo("Starting save_states.py")
    save_states()

