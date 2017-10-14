#!/usr/bin/env python
import rospy
import sys
from geometry_msgs.msg import Twist


class Program:
    def main(self):
        if len(sys.argv) < 5:
            raise ValueError("Need two argument to run the program, those argument are <from> <to>")
            
        topic_from = sys.argv[1]
        topic_to = sys.argv[2]
        
        self._pub = rospy.Publisher(topic_to, Twist, queue_size=10)
        rospy.Subscriber(topic_from, Twist, self.callback, queue_size=10)

    def callback(self, msg):
        self._pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('driver_remaper')
    try:
        Program().main()
        rospy.spin()
    except ValueError as error:
        rospy.logerr(error.message)
