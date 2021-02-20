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
group_name = "arm"
move_group = moveit_commander.MoveGroupCommander(group_name)


joy_arm = [0,0,0,0,0,0,0,0]
target_position = [0,0,0]
def joy_callback(joy_msg):
    global joy_arm
    global joy_hand
    joy_arm = joy_msg.axes
    # joy_hand = joy_msg.buttons   
        

def main():
    global joy_arm
    global target_position
    rate = rospy.Rate(10) # 10hz
    
    end_effector_link = move_group.get_end_effector_link()
    xyz = move_group.get_current_pose(end_effector_link=end_effector_link)
    target_position[0] = xyz.pose.position.x 
    target_position[1] = xyz.pose.position.y 
    target_position[2] = xyz.pose.position.z 
    # rospy.loginfo("\nEnd_effector_link: %s\n",end_effector_link)

    while not rospy.is_shutdown(): 

        rospy.Subscriber("joy", Joy, joy_callback)
        
        xyz = move_group.get_current_pose(end_effector_link=end_effector_link)
        target_orientation = [0.0, math.pi/2, 0.0]
        target_position[0] = xyz.pose.position.x + joy_arm[7]*0.05
        target_position[1] = xyz.pose.position.y + joy_arm[6]*0.05
        target_position[2] = xyz.pose.position.z + joy_arm[1]*0.1
        rospy.loginfo("\nXXXXXXXXXX: %s\n",target_position)
        target_pose = target_position + target_orientation
        
        move_group.set_pose_target(target_pose, end_effector_link)
        # move_group.go(wait=True)
        move_group.go()
        # rospy.loginfo("\nJoint Values: %s\n",target_xyz.pose.position)
        
        move_group.clear_pose_targets()
        # rate.sleep()
    rospy.spin()

if __name__ == "__main__":
    main()