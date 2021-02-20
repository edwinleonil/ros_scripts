#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

def talker():
    pub_joints = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('joint_state_publisher')
    rate = rospy.Rate(10) # 10hz
    joint_states = JointState()
    joint_states.header = Header()
    joint_states.header.stamp.secs = rospy.get_time()
    joint_states.name = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"]
    joint_states.position = [0.0, -0.3, -0.45, -2.8, -0.3, 2.1, 0.5]
    joint_states.velocity = []
    joint_states.effort = []
    rospy.loginfo(joint_states.header.stamp)
    

    while not rospy.is_shutdown():
      joint_states.header.stamp.secs = rospy.get_time()
      pub_joints.publish(joint_states)
      rate.sleep()



if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
