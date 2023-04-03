#!/usr/bin/env python
import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose

from std_msgs.msg import Int32 # Messages used in the node must be imported.



rospy.init_node('subscriber_py') #initialzing the node with name "subscriber_py"
# moveit start

moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "arm"
group = moveit_commander.MoveGroupCommander(group_name)
group.set_num_planning_attempts(3)
group.set_planning_time(5)


w = 0.658918
x = -0.22306
y = -0.1300
z = -0.035
group.set_planner_id('RRTConnect')

hand_group = moveit_commander.MoveGroupCommander("hand")
hand_group.set_num_planning_attempts(3)
hand_group.set_planning_time(15)

group.set_named_target("home")
plan = group.plan()
group.execute(plan, wait = True)
group.stop()
group.clear_pose_targets()
group.set_position_target([x, y, z])
plan = group.plan()
group.execute(plan, wait = True)
group.stop()
group.clear_pose_targets()

hand_group.set_named_target("grab_open")
hand_group.go(wait=True)
hand_group.stop()

hand_group.set_named_target("grab_close")
hand_group.go(wait=True)
hand_group.stop()

group.set_named_target("home")
## Now, we call the planner to compute the plan and execute it.
group.go(wait = True)
group.stop()

group.set_named_target("drop")
## Now, we call the planner to compute the plan and execute it.
group.go(wait = True)
group.stop()

hand_group.set_named_target("drop_open")
hand_group.go(wait=True)
hand_group.stop()

group.set_named_target("home")
## Now, we call the planner to compute the plan and execute it.
group.go(wait = True)
group.stop()


rospy.spin()