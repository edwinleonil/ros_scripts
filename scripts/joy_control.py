#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import JointState, Joy
# from sensor_msgs.msg import Joy
from std_msgs.msg import Header

joints = [0,0,0,0,0,0,0,0]
joy_data = [0,0,0,0,0,0,0,0]
def joy_callback(joy_msg):
    global joy_data
    joy_data = joy_msg.axes
    # rospy.loginfo(joy_data)
      

def joint_states_callback(states_msg):
    global joints
    # rate = rospy.Rate(10) # 10hz
    joints = list(states_msg.position[0:8])
    # rospy.loginfo(joints)

# Intializes everything
def main():
    
    # publishing to "turtle1/cmd_vel" to control turtle1
    global pub
    global joints
    # rospy.loginfo(joints)
    rospy.init_node('Joy2robot')
    joint_states = JointState()
    rate = rospy.Rate(5) # 10hz
    pub = rospy.Publisher("joint_states", JointState, queue_size=10)

       
    # joint_states.position = [0,0,0,0,0,0,0,0]                                                    
    while not rospy.is_shutdown(): 
        # subscribed to joystick inputs on topic "joy"
        rospy.Subscriber("joy", Joy, joy_callback)
        rospy.Subscriber("joint_states", JointState, joint_states_callback)

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

        joint_states.position = [joints[0] + joy_data[0]*0.05,
                                 joints[1] + joy_data[1]*0.05,
                                 joints[2],
                                 joints[3] + joy_data[1]*0.05,
                                 joints[4],
                                 joints[5] + joy_data[1]*0.05,
                                 joints[6], 
                                 joints[7]] 
        joint_states.velocity = []
        joint_states.effort = []

        pub.publish(joint_states)

        rospy.loginfo(joint_states.position)
        # rospy.spin()
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass