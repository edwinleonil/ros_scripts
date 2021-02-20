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
group_name = "panda_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)


joy_arm = [0,0,0,0,0,0,0]

def joy_callback(joy_msg):
    global joy_arm
    global joy_hand
    joy_arm = joy_msg.axes
    # joy_hand = joy_msg.buttons   
        

def main():
    global joy_arm

    rate = rospy.Rate(10) # 10hz
    joint = move_group.get_current_joint_values()
    # joint_names = ['panda_joint1', 
    #             'panda_joint2', 
    #             'panda_joint3', 
    #             'panda_joint4', 
    #             'panda_joint5', 
    #             'panda_joint6', 
    #             'panda_joint7'] 
    while not rospy.is_shutdown(): 

        rospy.Subscriber("joy", Joy, joy_callback)
        current_joint_val = move_group.get_current_joint_values()
        rospy.loginfo("\nJoint Values: %s\n",joy_arm)
        joint[0] = current_joint_val[0] + joy_arm[0]*0.1
        joint[1] = current_joint_val[1] + joy_arm[1]*0.1
        joint[2] = current_joint_val[2] + joy_arm[0]*0.1
        joint[3] = current_joint_val[3] + joy_arm[1]*0.1
        joint[4] = current_joint_val[4] + joy_arm[0]*0.1
        joint[5] = current_joint_val[5] + joy_arm[1]*0.1
        joint[6] = current_joint_val[6] + joy_arm[0]*0.1
        # rospy.loginfo("\nJoint Values: %s\n",joint)
        # rospy.loginfo("\nJoint Values: %s\n",val)
        move_to = move_group.set_joint_value_target(joint)
        
        move_group.go(move_to, wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
        # rate.sleep()
    rospy.spin()

if __name__ == "__main__":
    main()