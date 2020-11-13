#! /usr/bin/env python

import rospy
import actionlib
from actions.msgs import timerAction, timerGoal, timerResult

rospy.init_node('timer_action_client')
client = actionlib.SimpleActionClient('timer', timerAction)
client.wait_for_server()
goal = timerGoal()
goal.timer_to_wait = rospy.Duration.from_sec(7.0)
client.send_goal(goal)
client.wait_for_result()
print('Time elapsed: %f'%(client.get_result().time_elapsed.to_sec()))
