#!/usr/bin/env python
import rospy
from actionlib import SimpleActionClient, GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


def main():
    rospy.init_node("simple_navigation_goals")
    move_base_client = SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo('Connecting to server')
    move_base_client.wait_for_server()

    goal = MoveBaseGoal()

    goal.target_pose.header.frame_id = 'komodo_1/base_link'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = -1.0
    goal.target_pose.pose.orientation.w = 1.0

    rospy.loginfo('Sending goal')
    move_base_client.send_goal(goal)
    move_base_client.wait_for_result()

    if move_base_client.get_state() == GoalStatus.SUCCEEDED:
        rospy.loginfo('Hooray, the base moved 1 meter forward')
    else:
        rospy.loginfo('The base failed to move forward 1 meter for some reason')


if __name__ == '__main__':
    main()