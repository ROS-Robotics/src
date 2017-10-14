#!/usr/bin/env python
import roslib; roslib.load_manifest('ric_robot')
import rospy
from std_msgs.msg import Float64
 
 
def move_arm():
	pub_base_rotation=rospy.Publisher('/komodo_1/base_rotation_controller/command', Float64)
	pub_shoulder = rospy.Publisher('/komodo_1/shoulder_controller/command', Float64)
	pub_elbow1 = rospy.Publisher('/komodo_1/elbow1_controller/command', Float64)	
	pub_elbow2 = rospy.Publisher('/komodo_1/elbow2_controller/command', Float64)	
	pub_wrist = rospy.Publisher('/komodo_1/wrist_controller/command', Float64)
	pub_left_finger = rospy.Publisher('/komodo_1/left_finger_controller/command', Float64)
	pub_right_finger = rospy.Publisher('/komodo_1/right_finger_controller/command', Float64)
	rospy.init_node('move_arm')
	#while not rospy.is_shutdown():
	br = Float64(0.0)
	sh = Float64(0.0)
	e1 = Float64(0.0)
	e2 = Float64(0.0)	
  	wr = Float64(0.0) 
	lf = Float64(0.0) 	
	rf = Float64(0.0)
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
	rospy.sleep(5.0)

 
if __name__ == '__main__':
	try:
		move_arm()
	except rospy.ROSInterruptException:
		pass
