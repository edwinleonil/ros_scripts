#!/usr/bin/env python

import copy
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
from std_msgs.msg import Float32, String
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
import tf


target_position = [0,0,0]
target_pose=[0,0,0]
class MoveGroupPythonInteface(object):
    """MoveGroupPythonInteface"""
    def __init__(self):
        super(MoveGroupPythonInteface, self).__init__()
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface', anonymous=True)
        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()
        
        print ("============ Available Planning Groups:", robot.get_group_names())
        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        arm_group = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(arm_group)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        # print "============ Planning frame: %s" % planning_frame

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        # print "============ End effector link: %s" % eef_link

        # We can get a list of all the groups in the robot:
        # robot.get_group_names()
        # print ("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        # print "============ Printing robot state"
        # print robot.get_current_state()
        # print ""
        ## END_SUB_TUTORIAL

        # Misc variables
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        # self.arm_groups = arm_groups
        # self.target_position = [0,0,0]


    def set_max_velocity(self):
        max_velocity = 1
        rospy.loginfo("\nSet max velocity to: %s", max_velocity)
        self.move_group.set_max_velocity_scaling_factor(max_velocity)

    def set_planning_time(self):
        planning_time = 5  # in seconds
        self.move_group.set_planning_time(planning_time)  # in seconds
        rospy.loginfo("\nSet planning_time to: %s", planning_time)

    def get_planning_time(self):
        planning_time = self.move_group.get_planning_time()
        rospy.loginfo("\nPlanning_time set: %s", planning_time)

    def set_num_planning_attempts(self):
        num_planning_attempts = 10
        self.move_group.set_num_planning_attempts(num_planning_attempts)
        rospy.loginfo("\nNumber of planning attempts: %s", num_planning_attempts)

    def get_current_pose(self):
        current_pose = self.move_group.get_current_pose()
        rospy.loginfo("Current position:\n%s\n", current_pose)

    def get_active_joints(self):
        active_joints = self.move_group.get_active_joints()
        rospy.loginfo("Active Joints:\n%s\n", active_joints)

    def get_eef_link(self):
        eef_link = self.move_group.get_end_effector_link()
        rospy.loginfo("eef link:\n%s\n", eef_link)

    def get_current_rpy(self):
        current_rpy = self.move_group.get_current_rpy()
        rospy.loginfo("Current rpy:\n%s\n", current_rpy)

    def get_joints(self):
        get_joints = self.move_group.get_joints()
        rospy.loginfo("\nGet Joints: %s\n", get_joints)

    def get_joint_Values(self):
        current_joint_val = self.move_group.get_current_joint_values()
        rospy.loginfo("Joint Values:\n%s\n",current_joint_val)

    def move_to_xyz_target(self):
        move_group = self.move_group
        eef_link = move_group.get_end_effector_link()

        xyz_goal = [0.4, 0.0, 0.5]
        rospy.loginfo("Position Values:\n%s\n",xyz_goal)

        move_group.set_position_target(xyz_goal, eef_link)
        ## Now, call the planner to compute the plan and execute it.
        plan = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # Always good to clear the targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        move_group.clear_pose_targets()


    def move_to_pose_target(self):
        move_group = self.move_group
        pose_goal = geometry_msgs.msg.Pose()
        eef_link = move_group.get_end_effector_link()
        q_orientation = tf.transformations.quaternion_from_euler(0.0, math.pi/2, 0.0)

        pose_goal.position.x = 0.0
        pose_goal.position.y = 0.5
        pose_goal.position.z = 0.5
        pose_goal.orientation.x = q_orientation[0]
        pose_goal.orientation.y = q_orientation[1]
        pose_goal.orientation.z = q_orientation[2]
        pose_goal.orientation.w = q_orientation[3]
        rospy.loginfo("Pose Values:\n%s\n",pose_goal)

        move_group.set_pose_target(pose_goal)
        ## Now, call the planner to compute the plan and execute it.
        plan = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # Always good to clear the targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        move_group.clear_pose_targets()

    def go_to_joint_state(self):
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()

        joint_goal[0] = 0
        joint_goal[1] = -pi/4
        joint_goal[2] = 0
        joint_goal[3] = -pi/2
        joint_goal[4] = 0
        joint_goal[5] = pi/3
        joint_goal[6] = 0

        move_group.go(joint_goal, wait=True)
        move_group.stop()

    def close_hand(self):
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        for i in range(34):
            joint_goal[i] = 0.5
        move_group.go(joint_goal, wait=True)
        move_group.stop()

    def open_hand(self):
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        for i in range(34):
            joint_goal[i] = 0.0
        move_group.go(joint_goal, wait=True)
        move_group.stop()    

    # how to move to a goal state define in moveit
    # def close_hand(self):
    #     move_group = self.move_group
    #     move_group.set_pose_target()
    #     move_group.stop()
    #     move_group.clear_pose_targets()


def main():
  try:
    moveit_command = MoveGroupPythonInteface()
    # moveit_command.set_max_velocity()
    # moveit_command.set_planning_time()
    # moveit_command.get_planning_time()
    # moveit_command.set_num_planning_attempts()
    # moveit_command.go_to_joint_state()
    # moveit_command.get_current_pose()
    # moveit_command.get_active_joints()
    # moveit_command.get_eef_link()
    # moveit_command.get_current_rpy()
    # moveit_command.get_joint_Values()
    # moveit_command.move_to_xyz_target()
    # moveit_command.move_to_pose_target()
    # moveit_command.get_joints()
    # moveit_command.close_hand()
    # moveit_command.open_hand()
    # moveit_command.get_joint_Values()
    print("============ Request process complete!")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return


if __name__ == "__main__":
    main()