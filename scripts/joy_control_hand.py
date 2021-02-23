#!/usr/bin/env python
import sys
import math
import numpy as np
import moveit_commander
import rospy
from moveit_commander.conversions import pose_to_list
from sensor_msgs.msg import Joy


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python_interface", anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "hand"
move_group = moveit_commander.MoveGroupCommander(group_name)


joy_hand = [0,0,0,0,0,0]

def joy_callback(joy_msg):
    global joy_arm
    global joy_hand
    joy_hand = joy_msg.axes
    # joy_hand = joy_msg.buttons   
        

def main():
    global joy_hand
    val = 0
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown(): 

        rospy.Subscriber("joy", Joy, joy_callback)
        current_joint_val = move_group.get_current_joint_values()
        # rospy.loginfo("\nJoint Values: %s\n",current_joint_val)
        val = current_joint_val[28] + joy_hand[1]*0.1
        if val > 0.58:
            val = 0.58
        if val < 0.01:
            val = 0.01
        # rospy.loginfo("\nJoint Values: %s\n",val)
        current_joint_val = move_group.set_joint_value_target("qbhand_synergy_joint",val)
        move_group.go(current_joint_val, wait=True)
        rate.sleep()
    rospy.spin()

if __name__ == "__main__":
    main()