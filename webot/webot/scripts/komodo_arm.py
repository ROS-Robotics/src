#!/usr/bin/env python
import roslib; roslib.load_manifest('ric_robot')
import rospy
import math
import sys
from dynamixel_msgs.msg import JointState as dxl_JointState
from sensor_msgs.msg import JointState
from math import *
from ric_robot.srv import *
from ric_robot.msg import *
from std_msgs.msg import Float64

def br_callback(data):       
    global msg,ns
    msg.name[0]=ns+"_"+data.name
    msg.position[0]=data.current_pos
    msg.velocity[0]=data.velocity
    msg.effort[0]=data.load

def sh_callback(data):       
    global msg,ns
    msg.name[1]=ns+"_"+data.name
    msg.position[1]=data.current_pos
    msg.velocity[1]=data.velocity
    msg.effort[1]=data.load

def e1_callback(data):       
    global msg,ns
    msg.name[2]=ns+"_"+data.name
    msg.position[2]=data.current_pos
    msg.velocity[2]=data.velocity
    msg.effort[2]=data.load

def e2_callback(data):       
    global msg,ns
    msg.name[3]=ns+"_"+data.name
    msg.position[3]=data.current_pos
    msg.velocity[3]=data.velocity
    msg.effort[3]=data.load

def wr_callback(data):       
    global msg,ns
    msg.name[4]=ns+"_"+data.name
    msg.position[4]=data.current_pos
    msg.velocity[4]=data.velocity
    msg.effort[4]=data.load

def lf_callback(data):       
    global msg,ns
    msg.name[5]=ns+"_"+data.name
    msg.position[5]=data.current_pos
    msg.velocity[5]=data.velocity
    msg.effort[5]=data.load

def rf_callback(data):       
    global msg,ns
    msg.name[6]=ns+"_"+data.name
    msg.position[6]=data.current_pos
    msg.velocity[6]=data.velocity
    msg.effort[6]=data.load

def elev_callback(data):       
    global msg,pre_pos,pre_epos,final_epos,ns
    msg.name[7]=ns+"_"+data.name
    epos=data.current_pos*elev_rad2m
    msg.velocity[7]=data.velocity*elev_rad2m
    msg.effort[7]=data.load
    dp=epos-pre_epos
    if dp>0.0005:
       final_epos-=math.pi*2*elev_rad2m
    elif dp<-0.0005:
       final_epos+=math.pi*2*elev_rad2m
    pre_epos=epos
    final_epos+=dp
    msg.position[7]=final_epos

def handle_elev_set(req):
    global final_epos,elevpub,home_ok,elev_move
    if (home_ok==1) or (home_ok==0): 
       home_ok=2
       rospy.loginfo("Homing is done")
    elevpub.publish(0.0)
    elev_move=False
       #rospy.sleep(0.5)
    final_epos=req.pos
    rospy.loginfo("Elevator position is: %.3f",final_epos)
    return True

def epos_callback(data): 
    global elev_goal_pos,elev_move,elevpub,final_epos,epos_tol,max_elev_speed,home_ok, min_elev_pos,max_elev_pos
    if home_ok==2:
       elev_goal_pos=data.pos
       if elev_goal_pos>(max_elev_pos+0.000001) or elev_goal_pos<(min_elev_pos-0.000001):
          rospy.loginfo("Required goal (%.3f) is outside of the elevator range of motion [%.3f-%.3f]",elev_goal_pos,min_elev_pos,max_elev_pos)
       else:
          epos_err=elev_goal_pos-final_epos
          if abs(epos_err)>epos_tol:
             elev_move=True
    elif home_ok==1:
       rospy.logwarn("Cannot accept commands! Home service is running")
    elif home_ok==0:
       rospy.logwarn("Cannot accept commands! Please run the home service first")


def handle_elev_home(req):
    global home_ok,elev_move
    if (req.dir>0):
      if (final_epos<max_elev_pos+0.005):
        elev_move=False
        elevpub.publish(max_elev_speed/elev_rad2m)
        home_ok=1
        rospy.loginfo("Elevator is homing up")    
        return True
      else:
	return False
    elif(req.dir<0):
      if (final_epos>min_elev_pos-0.005):
        elev_move=False
        elevpub.publish(-max_elev_speed/elev_rad2m)
        home_ok=1
        rospy.loginfo("Elevator is homing down")
        return True
      else:
	return False
    else:
       rospy.loginfo("Please choose homing direction (1 - up / -1 - down)")
       return False
       
       
    
def komodo_arm():
    global pub,msg,ns
    global elev_rad2m
    global pre_epos,final_epos,elev_goal_pos,elev_move,epos_tol,max_elev_speed, min_elev_pos,max_elev_pos,elevpub
    global home_ok
    fr=0
    ns=rospy.get_namespace()
    ns=ns[1:-1]
    elev_rad2m=0.00175/(math.pi*2)
    min_elev_pos=0.0 #TODO: get as parameter?
    max_elev_pos=0.4 #TODO: get as parameter?
    home_ok=0 #0-not homed, 1-homing  2-homed
    epos_tol=0.001
    max_elev_speed=0.01
    elev_goal_pos=0
    elev_move=False
    pre_epos=0
    final_epos=0
    msg = JointState()
    rospy.init_node('komodo_arm', anonymous=True)
    have_elevator=rospy.get_param('have_elevator')
    for i in range(7):
        msg.name.append("")
        msg.position.append(0.0)
	msg.velocity.append(0.0)
	msg.effort.append(0.0)
    if have_elevator:
       msg.name.append("")
       msg.position.append(0.0)
       msg.velocity.append(0.0)
       msg.effort.append(0.0)
       rospy.Subscriber("/"+ns+"/elevator_controller/state", dxl_JointState, elev_callback)
       elevpub = rospy.Publisher("/"+ns+"/elevator_controller/command", Float64, queue_size=1)
       set_serv = rospy.Service("/"+ns+"/elevator_controller/set_position", set_elevator, handle_elev_set)
       home_serv = rospy.Service("/"+ns+"/elevator_controller/home", home_elevator, handle_elev_home)
       rospy.Subscriber("/"+ns+"/elevator_controller/pos_command", ric_elevator_command, epos_callback)
       rospy.loginfo("Seting up elevator...")
    rospy.Subscriber("/"+ns+"/base_rotation_controller/state", dxl_JointState, br_callback)
    rospy.Subscriber("/"+ns+"/shoulder_controller/state", dxl_JointState, sh_callback)
    rospy.Subscriber("/"+ns+"/elbow1_controller/state", dxl_JointState, e1_callback)
    rospy.Subscriber("/"+ns+"/elbow2_controller/state", dxl_JointState, e2_callback)
    rospy.Subscriber("/"+ns+"/left_finger_controller/state", dxl_JointState, lf_callback)
    rospy.Subscriber("/"+ns+"/right_finger_controller/state", dxl_JointState, rf_callback)
    rospy.Subscriber("/"+ns+"/wrist_controller/state", dxl_JointState, wr_callback)
    pub = rospy.Publisher('/joint_states', JointState, queue_size=1)
   
    while not rospy.is_shutdown():
    	msg.header.stamp = rospy.Time.now()
    	pub.publish(msg)
    	#rospy.loginfo("Elevator position is: %.3f",final_epos)
        if have_elevator:
           epos_err=elev_goal_pos-final_epos
           if elev_move==True and abs(epos_err)<epos_tol:
              elev_move=False
              rospy.loginfo("Elevator reached goal position: %.3f",elev_goal_pos)
              elevpub.publish(0.0)
           elif elev_move==True:
              if epos_err>0: speed=max_elev_speed
              elif epos_err<0: speed=-max_elev_speed
              elevpub.publish(speed/elev_rad2m)
	rospy.sleep(0.05)


if __name__ == '__main__':
    komodo_arm()

