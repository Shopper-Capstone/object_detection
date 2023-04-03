#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Vector3

class Pick_and_Place:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python", anonymous=True)


        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "arm" 
        group = moveit_commander.MoveGroupCommander(group_name)
        group.set_num_planning_attempts(3)
        group.set_planning_time(15)

        self.hand_group = moveit_commander.MoveGroupCommander("hand")
        self.hand_group.set_num_planning_attempts(3)
        self.hand_group.set_planning_time(15)

        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^

        # We can also print the name of the end-effector link for this group:
        eef_link = group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        self.move_group = group
        self.eef_link = eef_link

        self.move_group.clear_pose_targets()
        self.move_group.set_named_target("home")
        ## Now, we call the planner to compute the plan and execute it.
        self.move_group.go()
        self.move_group.stop()

        self.sub = rospy.Subscriber("/object_pos_to_base_link", Vector3, self.goToPoseGoalCallback)



    def goToPoseGoalCallback(self, data):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.

        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:

            
        x = round(data.x,4)
        y = round(data.y,4)
        z = round(data.z,4)+0.035

        self.move_group.set_planner_id('RRTConnect')
        self.move_group.set_position_target([x, y, z])
        plan = self.move_group.plan()
        self.move_group.execute(plan, wait = True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        self.hand_group.set_named_target("grab_open")
        self.hand_group.go(wait=True)
        self.hand_group.stop()

        self.hand_group.set_named_target("grab_close")
        self.hand_group.go(wait=True)
        self.hand_group.stop()

        self.move_group.set_named_target("home")
        ## Now, we call the planner to compute the plan and execute it.
        self.move_group.go(wait = True)
        self.move_group.stop()

        self.move_group.set_named_target("drop")
        ## Now, we call the planner to compute the plan and execute it.
        self.move_group.go(wait = True)
        self.move_group.stop()
        
        self.hand_group.set_named_target("drop_open")
        self.hand_group.go(wait=True)
        self.hand_group.stop()

        self.move_group.set_named_target("home")
        ## Now, we call the planner to compute the plan and execute it.
        self.move_group.go(wait = True)
        self.move_group.stop()


        #print(self.move_group.get_current_pose().pose)
        # self.move_group.set_named_target("home")
        # ## Now, we call the planner to compute the plan and execute it.
        # plan = self.move_group.go(wait=True)
        # # Calling `stop()` ensures that there is no residual movement
        # self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()

if __name__ == '__main__':
    arm = Pick_and_Place()
    rospy.spin()
