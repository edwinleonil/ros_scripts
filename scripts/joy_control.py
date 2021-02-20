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
    rospy.loginfo(joints)

# Intializes everything
def main():
    
    # publishing to "turtle1/cmd_vel" to control turtle1
    global pub
    global joints
    global joy_data
    # rospy.loginfo(joints)
    rospy.init_node('Joy2robot')
    joint_states = JointState()
    rate = rospy.Rate(5) # 10hz
    pub = rospy.Publisher("/move_group/fake_controller_joint_states", JointState, queue_size=10)
    # pub = rospy.Publisher("joint_states", JointState, queue_size=10)
    joint_states.position =  joints[0:6]     
                                                      
    while not rospy.is_shutdown(): 
        # subscribed to joystick inputs on topic "joy"
        rospy.Subscriber("joy", Joy, joy_callback)
        # rospy.Subscriber("joint_states", JointState, joint_states_callback)
        rospy.Subscriber("joint_states", JointState, joint_states_callback)
        
        joint_states.header = Header()
        joint_states.header.stamp.secs = rospy.get_time()
        joint_states.name = ['panda_joint1', 
                            'panda_joint2', 
                            'panda_joint3', 
                            'panda_joint4', 
                            'panda_joint5', 
                            'panda_joint6', 
                            'panda_joint7']                   

        joint_states.position = [joints[0] + joy_data[0]*0.01,
                                 joints[1] + joy_data[1]*0.01,
                                 joints[2],
                                 joints[3],
                                 joints[4],
                                 joints[5], 
                                 joints[6]] 
        joint_states.velocity = []
        joint_states.effort = []

        pub.publish(joint_states)
        rate.sleep()
        # rospy.loginfo(joint_states.position)
        # rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass