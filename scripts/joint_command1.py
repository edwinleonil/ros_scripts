#!/usr/bin/env python

import rospy
from math import pi
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

def talker():
    pub_joints = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('joint_state_publisher')
    rate = rospy.Rate(10) # 10hz
    joint_states = JointState()
    joint_states.header = Header()
    joint_states.header.stamp.secs = rospy.get_time()
    joint_states.name = ["joint_1", 
                         "joint_2", 
                         "joint_3", 
                         "joint_4", 
                         "joint_5", 
                         "joint_6", 
                         "joint_7", 
                         "qbhand_synergy_joint"]
    joint_states.position = [0.0, 
                             0.0, 
                             0.0, 
                             pi/2, 
                             0.0, 
                             pi/2, 
                             0.0, 
                             0.3]
    joint_states.velocity = []
    joint_states.effort = []
    rospy.loginfo(joint_states.header.stamp)
    
    while not rospy.is_shutdown():
      joint_states.header.stamp.secs = rospy.get_time()
      pub_joints.publish(joint_states)
      rate.sleep()
    #   rospy.spin()



if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
