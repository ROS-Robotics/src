#!/usr/bin/env python
__author__ = 'tom'
import rospy
from std_msgs.msg import Float64


def menu():
    return '\nPlease select a single number from the option below to move the komodo arm:\n1) Move to home position.\n2) Say hi.\n3) Rotate wrist.\n4) Prepare to hit position. (not the hit itself just to prepare)\n5) Hit!.\n6) somthing.\n0) To exit this program.'


def main():
    rospy.init_node("arm_move")
    komodo_arm_publishers = [rospy.Publisher('/komodo_1/wrist_controller/command', Float64, queue_size=1),
                             rospy.Publisher('/komodo_1/shoulder_controller/command', Float64, queue_size=1),
                             rospy.Publisher('/komodo_1/right_finger_controller/command', Float64, queue_size=1),
                             rospy.Publisher('/komodo_1/left_finger_controller/command', Float64, queue_size=1),
                             rospy.Publisher('/komodo_1/elbow2_controller/command', Float64, queue_size=1),
                             rospy.Publisher('/komodo_1/elbow1_controller/command', Float64, queue_size=1),
                             rospy.Publisher('/komodo_1/base_rotation_controller/command', Float64, queue_size=1),
                             ]
    choice = -1
    try:
        while choice != 0 and not rospy.is_shutdown():
            print menu()
            try:
                choice = int(raw_input('>'))
                if choice == 1:
                    komodo_arm_positions = [Float64()] * len(komodo_arm_publishers)
                    for i in xrange(len(komodo_arm_publishers)):
                        komodo_arm_positions[i].data = 0.0
                        komodo_arm_publishers[i].publish(komodo_arm_positions[i])
                elif choice == 2:
                    komodo_arm_positions = [Float64()] * len(komodo_arm_publishers)
                    komodo_arm_positions[0].data = 0.0
                    komodo_arm_publishers[0].publish(komodo_arm_positions[0])
                    komodo_arm_positions[1].data = 0.0
                    komodo_arm_publishers[1].publish(komodo_arm_positions[1])
                    komodo_arm_positions[2].data = 0.0
                    komodo_arm_publishers[2].publish(komodo_arm_positions[2])
                    komodo_arm_positions[3].data = 0.0
                    komodo_arm_publishers[3].publish(komodo_arm_positions[3])
                    komodo_arm_positions[4].data = 0.0
                    komodo_arm_publishers[4].publish(komodo_arm_positions[4])
                    komodo_arm_positions[5].data = 0.0
                    komodo_arm_publishers[5].publish(komodo_arm_positions[5])
                    komodo_arm_positions[6].data = 0.0
                    komodo_arm_publishers[6].publish(komodo_arm_positions[6])
                    rospy.sleep(3.0)

                    komodo_arm_positions = [Float64()] * len(komodo_arm_publishers)
                    komodo_arm_positions[0].data = 0.0
                    komodo_arm_publishers[0].publish(komodo_arm_positions[0])
                    komodo_arm_positions[1].data = 0.0
                    komodo_arm_publishers[1].publish(komodo_arm_positions[1])
                    komodo_arm_positions[2].data = 0.0
                    komodo_arm_publishers[2].publish(komodo_arm_positions[2])
                    komodo_arm_positions[3].data = 0.0
                    komodo_arm_publishers[3].publish(komodo_arm_positions[3])
                    komodo_arm_positions[4].data = 0.0
                    komodo_arm_publishers[4].publish(komodo_arm_positions[4])
                    komodo_arm_positions[5].data = 0.5
                    komodo_arm_publishers[5].publish(komodo_arm_positions[5])
                    komodo_arm_positions[6].data = 0.0
                    komodo_arm_publishers[6].publish(komodo_arm_positions[6])
                    rospy.sleep(3.0)

                    komodo_arm_positions = [Float64()] * len(komodo_arm_publishers)
                    komodo_arm_positions[0].data = 0.0
                    komodo_arm_publishers[0].publish(komodo_arm_positions[0])
                    komodo_arm_positions[1].data = 0.0
                    komodo_arm_publishers[1].publish(komodo_arm_positions[1])
                    komodo_arm_positions[2].data = -0.5
                    komodo_arm_publishers[2].publish(komodo_arm_positions[2])
                    komodo_arm_positions[3].data = 0.5
                    komodo_arm_publishers[3].publish(komodo_arm_positions[3])
                    komodo_arm_positions[4].data = 0.0
                    komodo_arm_publishers[4].publish(komodo_arm_positions[4])
                    komodo_arm_positions[5].data = -0.5
                    komodo_arm_publishers[5].publish(komodo_arm_positions[5])
                    komodo_arm_positions[6].data = 0.0
                    komodo_arm_publishers[6].publish(komodo_arm_positions[6])
                    rospy.sleep(5.0)

                    komodo_arm_positions = [Float64()] * len(komodo_arm_publishers)
                    komodo_arm_positions[0].data = 0.0
                    komodo_arm_publishers[0].publish(komodo_arm_positions[0])
                    komodo_arm_positions[1].data = 0.0
                    komodo_arm_publishers[1].publish(komodo_arm_positions[1])
                    komodo_arm_positions[2].data = 0.0
                    komodo_arm_publishers[2].publish(komodo_arm_positions[2])
                    komodo_arm_positions[3].data = 0.0
                    komodo_arm_publishers[3].publish(komodo_arm_positions[3])
                    komodo_arm_positions[4].data = 0.0
                    komodo_arm_publishers[4].publish(komodo_arm_positions[4])
                    komodo_arm_positions[5].data = 0.0
                    komodo_arm_publishers[5].publish(komodo_arm_positions[5])
                    komodo_arm_positions[6].data = 0.0
                    komodo_arm_publishers[6].publish(komodo_arm_positions[6])
		elif choice == 3:
		    msg = Float64()
		    msg.data = 1.0
		    komodo_arm_publishers[0].publish(msg)
		    rospy.sleep(2.0)
		    msg = Float64()
		    msg.data = -1.0
		    komodo_arm_publishers[0].publish(msg)
		    rospy.sleep(2.0)

		elif choice == 4:
		    komodo_arm_positions = [Float64()] * len(komodo_arm_publishers)
                    komodo_arm_positions[0].data = 0.0
                    komodo_arm_publishers[0].publish(komodo_arm_positions[0])
                    komodo_arm_positions[1].data = 0.7
                    komodo_arm_publishers[1].publish(komodo_arm_positions[1])
                    komodo_arm_positions[2].data = -0.5
                    komodo_arm_publishers[2].publish(komodo_arm_positions[2])
                    komodo_arm_positions[3].data = 0.5
                    komodo_arm_publishers[3].publish(komodo_arm_positions[3])
                    komodo_arm_positions[4].data = 0.8
                    komodo_arm_publishers[4].publish(komodo_arm_positions[4])
                    komodo_arm_positions[5].data = 0.0
                    komodo_arm_publishers[5].publish(komodo_arm_positions[5])
                    komodo_arm_positions[6].data = -1.0
                    komodo_arm_publishers[6].publish(komodo_arm_positions[6])

                elif choice == 5:
                    msg = Float64()
                    msg.data = 1.0
                    komodo_arm_publishers[6].publish(msg)
		elif choice == 6:
    		    komodo_arm_positions = [Float64()] * len(komodo_arm_publishers)
                    komodo_arm_positions[0].data = 0.0
                    komodo_arm_publishers[0].publish(komodo_arm_positions[0])
                    komodo_arm_positions[1].data = 0.0
                    komodo_arm_publishers[1].publish(komodo_arm_positions[1])
                    komodo_arm_positions[2].data = 0.0
                    komodo_arm_publishers[2].publish(komodo_arm_positions[2])
                    komodo_arm_positions[3].data = 0.0
                    komodo_arm_publishers[3].publish(komodo_arm_positions[3])
                    komodo_arm_positions[4].data = 0.7
                    komodo_arm_publishers[4].publish(komodo_arm_positions[4])
                    komodo_arm_positions[5].data = 0.7
                    komodo_arm_publishers[5].publish(komodo_arm_positions[5])
		    rospy.sleep(1.0)
                    komodo_arm_positions[6].data = -1.0
                    komodo_arm_publishers[6].publish(komodo_arm_positions[6])
                elif choice != 0:
                    rospy.loginfo("Invalid choice, please enter your choice between 0-6.")

            except ValueError:
                rospy.logerr('Invalid Integer, this program only except integers as choices')
    except KeyboardInterrupt:
        pass
    finally:
        rospy.loginfo('Have a nice day')


if __name__ == '__main__':
    main()
