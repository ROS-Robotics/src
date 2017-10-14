#!/usr/bin/env python

__author__ = 'tom'
import rospy
import actionlib

from control_msgs.msg import FollowJointTrajectoryAction
from dynamixel_msgs.msg import JointState as JointStateDynamixel
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class JointSubscriber:
    def __init__(self, joint):
        rospy.Subscriber(joint + '/state', JointStateDynamixel, self.joint_state_cb)
	
        rospy.loginfo('Subscribing for %s joint state', joint)

        self.joint_name = joint
        self.state = JointStateDynamixel()
        self.received = False

    def joint_state_cb(self, msg):
        if not self.received:
            self.received = True
        self.state = msg

    def get_position(self):
        return self.state.current_pos


class JointCommander:
    def __init__(self, joint):
        self.joint_name = joint
        self.pub = rospy.Publisher(joint + '/command', Float64, queue_size=10)

    def command(self, pos):
        self.pub.publish(pos)


class FollowController:
    def __init__(self):
        self.ns = 'komodo_arm_controller'
        self.joints = rospy.get_param('dynamixel/arm_controller/arm_joints_name')
        rospy.loginfo('Configured for' + str(len(self.joints)) + 'joints')
        self.joints_pub = [JointCommander(name + '_controller') for name in self.joints]
        self.joints_names = []
	
        for idx in xrange((len(self.joints))):
            self.joints_names.append(self.joints[idx] + '_joint')

        self.name = self.ns + '/follow_joint_trajectory'
        self.server = actionlib.SimpleActionServer(self.name, FollowJointTrajectoryAction, execute_cb=self.actionCb,
                                                   auto_start=False)

        rospy.loginfo('Started FollowController (' + self.name + '). joints: ' + str(self.joints))

    def startup(self):
        self.server.start()

    def actionCb(self, goal):
        rospy.loginfo(self.name + ': Action goal received.')
        traj = goal.trajectory
        if set(self.joints_names) != set(traj.joint_names):
            print set(self.joints_names)
            print set(traj.joint_names)
            msg = "Trajectory joint does not match action controlled joints" + str(traj.joint_names)
            rospy.logerr(msg)
            self.server.set_aborted(text=msg)
            return
        if not traj.points:
            msg = "Trajectory empty"
            rospy.logerr(msg)
            self.server.set_aborted(text=msg)
            return

        try:
            indexes = [traj.joint_names.index(joint) for joint in self.joints_names]
        except ValueError as val:
            msg = 'Trajectory invalid'
            rospy.logerr(msg)
            self.server.set_aborted(text=msg)
            return
        if self.executeTrajectory(traj):
            self.server.set_succeeded()
            rospy.loginfo('Executed')
        else:
            rospy.logerr('Execution failed')
            self.server.set_aborted(text='Execution failed')

    def executeTrajectory(self, traj):
        rospy.loginfo('Executing trajectory with' + str(len((traj.points))) + 'points(s)')
        time = rospy.Time.now()
        start = traj.header.stamp

        try:
            indexes = [traj.joint_names.index(joint) for joint in self.joints_names]
        except ValueError as val:
            msg = 'Trajectory invalid'
            rospy.logerr(msg)
            self.server.set_aborted(text=msg)
            return False

        for point in traj.points:
            if self.server.is_preempt_requested():
                rospy.loginfo('Stopping arm movement')
                self.server.set_preempted()
                break
            while rospy.Time.now() + rospy.Duration(0.01) < start:
                rospy.sleep(0.01)

            desired = [point.positions[k] for k in indexes]
            endtime = start + point.time_from_start

            for i in xrange(len(self.joints)):
                self.joints_pub[i].command(desired[i])

            while rospy.Time.now() + rospy.Duration(0.01) < endtime:
                rospy.sleep(0.01)
        return True


if __name__ == '__main__':
    rospy.init_node('follow_joint_controller')
    rospy.loginfo('Follow action server')
    action_server = FollowController()
    action_server.startup()

    rospy.loginfo('Spinning')
    rospy.spin()
