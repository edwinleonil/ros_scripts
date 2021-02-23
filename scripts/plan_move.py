#!/usr/bin/env python

import copy
import time
import sys
import math
from math import pi
import numpy as np
import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
import rospy
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Point, Pose
import tf

       
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
move_arm = moveit_commander.MoveGroupCommander("arm")
move_hand = moveit_commander.MoveGroupCommander("hand")
display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path",
                                                    moveit_msgs.msg.DisplayTrajectory)
        
        
pose_goal = geometry_msgs.msg.Pose()
eef_link = move_arm.get_end_effector_link()
q_orientation = tf.transformations.quaternion_from_euler(math.pi, math.pi/4, 0.0)
# q_orientation = tf.transformations.quaternion_from_euler(0.0, math.pi/2, 0.0)

pose_goal.position.x = -0.2
pose_goal.position.y = 0.0
pose_goal.position.z = 0.7
pose_goal.orientation.x = q_orientation[0]
pose_goal.orientation.y = q_orientation[1]
pose_goal.orientation.z = q_orientation[2]
pose_goal.orientation.w = q_orientation[3]
rospy.loginfo("Pose Values:\n%s\n",pose_goal)


move_arm.set_pose_target(pose_goal)
## Now, call the planner to compute the plan and execute it.
# plan = move_group.go(wait=True)

plan1 = move_arm.plan()
print("============ Waiting while RVIZ displays plan1...")
rospy.sleep(5)

moveit_commander.roscpp_shutdown()
