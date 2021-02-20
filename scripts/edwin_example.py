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


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python_interface", anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "panda_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)


joint_goal = [0, 0, 0, 0, 0, 0]
current_xyz = [0, 0, 0]
target_position = [0, 0, 0]
curent_orientation = [0, 0, 0, 0]


def get_active_joints():
    active_joints = move_group.get_active_joints()
    rospy.loginfo("\nActive Joints: %s\n", active_joints)


def get_joints():
    get_joints = move_group.get_joints()
    rospy.loginfo("\nGet Joints: %s\n", get_joints)


def get_joint_Values():
    current_joint_val = move_group.get_current_joint_values()
    rospy.loginfo("\nJoint Values: %s\n",current_joint_val)


def get_current_pose():
    current_pose = move_group.get_current_pose(end_effector_link="panda_link6")
    current_xyz[0] = current_pose.pose.position.x
    current_xyz[1] = current_pose.pose.position.y
    current_xyz[2] = current_pose.pose.position.z
    curent_orientation[0] = current_pose.pose.orientation.x
    curent_orientation[1] = current_pose.pose.orientation.x
    curent_orientation[2] = current_pose.pose.orientation.x
    curent_orientation[3] = current_pose.pose.orientation.x

    rospy.loginfo("\nCurrent position: %s", current_xyz)
    rospy.loginfo("\nCurrent orientation: %s", curent_orientation)


def get_current_rpy():
    roll_pitch_yaw = move_group.get_current_rpy(end_effector_link="link_6")
    rospy.loginfo("\nCurrent Roll, Pitch and Yaw: %s", roll_pitch_yaw)


def get_end_effector_link():
    end_effector_link = move_group.get_end_effector_link()
    rospy.loginfo("\nEnd effector link: %s", end_effector_link)


def home_position():
    move_group.set_max_velocity_scaling_factor(1)
    joint_goal[0] = 0.0
    joint_goal[1] = 0.0
    joint_goal[2] = 0.0
    joint_goal[3] = 0.0
    joint_goal[4] = 0.0
    joint_goal[5] = 0.0
    move_group.go(joint_goal, wait=True)
    move_group.stop()
    move_group.clear_pose_targets()


#  Doesn't work properlly
def set_position_target():
    end_effector_link = move_group.get_end_effector_link()
    target_position[0] = 0.4
    target_position[1] = 0.1
    target_position[2] = 0.5
    move_group.set_position_target(target_position, end_effector_link)
    move_group.go(wait=True)

def set_pose_target_01():
    move_group.set_max_velocity_scaling_factor(1)
    target_position[0] = 0.4 # 0.4 - 0.3
    target_position[1] = 0.1 # 0.1 - 0.3
    target_position[2] = 0.5

    target_orientation = [0.0, math.pi/2, 0.0]

    target_pose = target_position + target_orientation
    rospy.loginfo("\ntarget pose: %s", target_pose)

    move_group.set_pose_target(
        target_pose, end_effector_link="tool_tcp")
    plan = move_group.plan()
    rospy.sleep(1)
    if plan.joint_trajectory.points:
        rospy.loginfo("\nPlanning was Successful\n")
        move_group.go(wait=True)
    else:
        rospy.logerr("\nPlanning was unsuccessful\n")
    move_group.stop()
    move_group.clear_pose_targets()

def set_pose_target_02():
    move_group.set_max_velocity_scaling_factor(1)
    target_position[0] = 0.4
    target_position[1] = 0.3
    target_position[2] = 0.5

    target_orientation = [0.0, math.pi/2, 0.0]

    target_pose = target_position + target_orientation
    rospy.loginfo("\ntarget pose: %s", target_pose)

    move_group.set_pose_target(
        target_pose, end_effector_link="grip3")
    plan = move_group.plan()
    rospy.sleep(1)
    if plan.joint_trajectory.points:
        rospy.loginfo("\nPlanning was Successful\n")
        move_group.go(wait=True)
    else:
        rospy.logerr("\nPlanning was unsuccessful\n")
    move_group.stop()
    move_group.clear_pose_targets()

def set_max_velocity():
    move_group.set_max_velocity_scaling_factor(1)


def set_planning_time():
    move_group.set_planning_time(5)  # in seconds


def get_planning_time():
    planning_time = move_group.get_planning_time()
    rospy.loginfo("\nplanning_time: %s", planning_time)


def set_num_planning_attempts():
    num_planning_attempts = move_group.set_num_planning_attempts(10)
    rospy.loginfo("\nplanning_time: %s", num_planning_attempts)


if __name__ == "__main__":
    while not rospy.is_shutdown():
        # set_max_velocity()
        # set_planning_time()
        # set_num_planning_attempts()
        # get_active_joints()
        # get_joints()
        # get_end_effector_link()
        # get_current_rpy()
        # get_joint_Values()
        # get_current_pose()
        # get_planning_time()
        
        # set_pose_target_01()  # go to target position xyz
        set_position_target()
        # set_pose_target_02()  # go to target position xyz
        # rospy.sleep(2)
        # home_position()  # return to home position
        # rospy.sleep(2)
        rospy.spin()
