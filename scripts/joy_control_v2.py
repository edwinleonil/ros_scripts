#!/usr/bin/env python
import sys
import math
import numpy as np
import moveit_commander
import rospy
from moveit_commander.conversions import pose_to_list
# from geometry_msgs.msg import Point, Pose
# from std_msgs.msg import Float32, String, Bool
from sensor_msgs.msg import Joy


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python_interface", anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "arm"
move_group = moveit_commander.MoveGroupCommander(group_name)
current_xyz = [0, 0, 0]
target_xyz = [0, 0, 0]
joy_data = [0, 0, 0]
joint_goal = [0, 0, 0, 0, 0, 0,0.0]
dv = 0.05
eef_link = move_group.get_end_effector_link()

def joy_callback(joy_msg):
    global joy_data
    joy_data = joy_msg.axes


def go_to_target_position():
    """""Get current position and move to next position"""
    global joy_data
    
    current_pose = move_group.get_current_pose()
    
    target_xyz[0] = current_pose.pose.position.x + joy_data[1]
    target_xyz[1] = current_pose.pose.position.y + joy_data[2]
    target_xyz[2] = current_pose.pose.position.z + 0.0
    
    # target_orientation = [math.pi, -math.pi/2, 0.0]

    # target_pose = target_xyz + target_orientation
    rospy.loginfo("Current arm position:\n%s\n", target_xyz)

    move_group.set_position_target(target_xyz, eef_link)
    # move_group.set_pose_target(
    #     target_pose, end_effector_link="end_effector_link")
    move_group.go(wait=True)
        

def main():

    # move_group.set_max_velocity_scaling_factor(1)
    rospy.Subscriber("joy", Joy, joy_callback)

    while not rospy.is_shutdown(): 
        go_to_target_position()
        
    
    rospy.spin()

if __name__ == "__main__":
    main()