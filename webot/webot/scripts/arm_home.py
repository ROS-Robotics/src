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
	rospy.init_node('move_arm_home')
	#while not rospy.is_shutdown():
	br = Float64(0.0)
	sh = Float64(0.0)
	e1 = Float64(0.0)
	e2 = Float64(0.0)	
  	wr = Float64(0.0) 
	lf = Float64(-0.3) 	
	rf = Float64(0.3)
	pub_base_rotation.publish(br)
	pub_shoulder.publish(sh)
	pub_elbow1.publish(e1)
	pub_elbow2.publish(e2)
	pub_wrist.publish(wr)
	pub_left_finger.publish(lf)
	pub_right_finger.publish(rf)
	rospy.sleep(0.1)
	pub_base_rotation.publish(br)
	pub_shoulder.publish(sh)
	pub_elbow1.publish(e1)
	pub_elbow2.publish(e2)
	pub_wrist.publish(wr)
	pub_left_finger.publish(lf)
	pub_right_finger.publish(rf)
	rospy.sleep(6.0)
	# 2nd instance 
        br = Float64(0)
	sh = Float64(1.08)
	e1 = Float64(0)
	e2 = Float64(2.08)	
  	wr = Float64(0) 
	lf = Float64(-1.1) 	
	rf = Float64(1.1)
	pub_base_rotation.publish(br)
	pub_shoulder.publish(sh)
	pub_elbow1.publish(e1)
	pub_elbow2.publish(e2)
	pub_wrist.publish(wr)
	pub_left_finger.publish(lf)
	pub_right_finger.publish(rf)
	rospy.sleep(5.0)
        # 3rd instance
        br = Float64(0)
	sh = Float64(1.2)
	e1 = Float64(0)
	e2 = Float64(1.93)	
  	wr = Float64(0) 
	lf = Float64(-1.1) 	
	rf = Float64(1.1)
	pub_base_rotation.publish(br)
	pub_shoulder.publish(sh)
	pub_elbow1.publish(e1)
	pub_elbow2.publish(e2)
	pub_wrist.publish(wr)
	pub_left_finger.publish(lf)
	pub_right_finger.publish(rf)
	
 
if __name__ == '__main__':
	try:
		move_arm()
	except rospy.ROSInterruptException:
		pass
