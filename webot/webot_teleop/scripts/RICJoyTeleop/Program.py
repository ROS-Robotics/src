#!/usr/bin/env python
__author__ = 'tom1231'
import pygame
from pygame.locals import *
import rospy
import sys
from threading import Thread, RLock
from geometry_msgs.msg import Twist

LEFT_RIGHT_NUM = int(sys.argv[6])
UP_DOWN_NUM = int(sys.argv[7])

BOOST_SPEED_BUTTON = int(sys.argv[8])
ENABLE_BUTTON = int(sys.argv[9])
JOY_NUM = int(sys.argv[10])

REV_UP_DOWN = int(sys.argv[11])
REV_LEFT_RIGHT = int(sys.argv[12])

class Program:
    def __init__(self):
        #rospy.wait_for_service('/devsOnline')
        self._axes = [0.0, 0.0]
        self._enable = False
        self._maxSpeedLinear = float(sys.argv[1])
        self._maxSpeedAngular = float(sys.argv[5])
        self._scale = float(sys.argv[2])
        self._factor = 1
        self._lock = RLock()

        rospy.init_node('ric_joyTeleop')
        self._pub = rospy.Publisher('%s' % sys.argv[3], Twist, queue_size=1)

        rospy.loginfo("listen to %s" % self._pub.name)

        Thread(target=self.listenToJoystick, args=()).start()

        pubHz = int(sys.argv[4])

        rate = rospy.Rate(pubHz)
        try:
            while not rospy.is_shutdown():
                if self.isEnable():
                    msg = Twist()

                    msg.linear.x = self.getUpAndDownAxes() * REV_UP_DOWN
                    msg.angular.z = self.getLeftAndRightAxes() * REV_LEFT_RIGHT

                    self._pub.publish(msg)
                rate.sleep()

        except KeyboardInterrupt: pass
        finally:
            pygame.event.post(pygame.event.Event(QUIT))

    def setLeftAndRightAxes(self, newAxis):
        with self._lock:
            self._axes[0] = newAxis

    def setUpAndDownAxes(self, newAxis):
        with self._lock:
            self._axes[1] = newAxis

    def getLeftAndRightAxes(self):
        with self._lock:
            return self._axes[LEFT_RIGHT_NUM] * self.getMaxSpeedAngular() * self.getFactor()

    def getUpAndDownAxes(self):
        with self._lock:
            return self._axes[UP_DOWN_NUM] * self.getMaxSpeedLinear() * self.getFactor()

    def setEnable(self, newVal):
        with self._lock:
            self._enable = newVal

    def isEnable(self):
        with self._lock:
            return self._enable

    def setFactor(self, val):
        with self._lock:
            self._factor = val

    def getFactor(self):
        with self._lock:
            return self._factor

    def getMaxSpeedLinear(self):
        with self._lock:
            return self._maxSpeedLinear

    def getMaxSpeedAngular(self):
        with self._lock:
            return self._maxSpeedAngular

    def listenToJoystick(self):
        pygame.init()
        pygame.joystick.init()
        try:
            joystick = pygame.joystick.Joystick(JOY_NUM)
            joystick.init()
            quitLoop = False

            while not quitLoop:
                event = pygame.event.wait()
                if event.type == QUIT:
                    quitLoop = True

                elif event.type == JOYAXISMOTION:
                    if self.isEnable():
			
                        data = event.dict
                        if data['axis'] == LEFT_RIGHT_NUM:
                            self.setLeftAndRightAxes(data['value'])
                        elif data['axis'] == UP_DOWN_NUM:
                            self.setUpAndDownAxes(data['value'])

                elif event.type == JOYBUTTONDOWN:
                    if event.dict['button'] == BOOST_SPEED_BUTTON:
                        self.setFactor(self._scale)
                    elif event.dict['button'] == ENABLE_BUTTON:
                        self.setEnable(True)

                elif event.type == JOYBUTTONUP:
                    if event.dict['button'] == BOOST_SPEED_BUTTON:
                        self.setFactor(1.0)
                    elif event.dict['button'] == ENABLE_BUTTON:
                        self.setEnable(False)
                        self.setLeftAndRightAxes(0.0)
                        self.setUpAndDownAxes(0.0)

                msg = Twist()

                msg.linear.x = self.getUpAndDownAxes() * REV_UP_DOWN
                msg.angular.z = self.getLeftAndRightAxes() * REV_LEFT_RIGHT

                self._pub.publish(msg)
        except:
            rospy.logerr("Joystick [%d] not found" % JOY_NUM)


if __name__ == '__main__':
    Program()
