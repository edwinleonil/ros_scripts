#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState, Joy
# from sensor_msgs.msg import Joy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Header

joy_control = [0,0,0,0,0,0,0,0]
max_hand_limit = 0.7
min_hand_limit = 0.001
def joy_callback(joy_msg):
    global joy_control
    joy_control = joy_msg.axes
    # rospy.loginfo(joy_data)
      
def main():
    
    global pub
    global hand_joint
    global joy_control

    rospy.init_node('hand_control')
    jointCmd = JointTrajectory()
    point = JointTrajectoryPoint()
    rospy.Subscriber("joy", Joy, joy_callback)
    pub = rospy.Publisher("/qbhand/control/qbhand_synergy_trajectory_controller/command", JointTrajectory, queue_size=10)
    jointCmd = JointTrajectory()  
    point = JointTrajectoryPoint()
    
    while not rospy.is_shutdown(): 
        
        jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0);  
        point.time_from_start = rospy.Duration.from_sec(5.0)
        
        jointCmd.joint_names.append("qbhand_synergy_joint")

        if joy_control[2] >= 0:
            joy_hand = abs(joy_control[2]-1)*0.5
        elif joy_control[2] <-0.001:
            joy_hand = 0.5 + abs(joy_control[2])*0.5 

        val = joy_hand
     
        if val > max_hand_limit:
            val = max_hand_limit
        if val < min_hand_limit:
            val = min_hand_limit

        velocity = 0
        point.positions.append(val)
        point.velocities.append(velocity)
        point.accelerations.append(0)
        point.effort.append(0) 

        jointCmd.points.append(point)
        rate = rospy.Rate(50)
        count = 0
        

        while (count < 50):
            pub.publish(jointCmd)
            count = count + 1
            rate.sleep() 

        # rospy.loginfo(jointCmd)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass