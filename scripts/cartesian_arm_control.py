#!/usr/bin/env python
###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2019 Kinova inc. All rights reserved.
#
# This software may be modified and distributed
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.

# xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
# This script has been adapted from the Kinova examples - By the AMRC UK
# xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
###

import sys
import rospy
import time
from kortex_driver.srv import *
from kortex_driver.msg import *
from sensor_msgs.msg import Joy

class ExampleCartesianActionsWithNotifications:
    def __init__(self):
        try:
            rospy.init_node('example_cartesian_poses_with_notifications_python')

            # Initialise lists to get joy data
            self.joy_control = [0, 0, 0, 0, 0, 0, 0, 0]
            self.joy_buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]


            self.HOME_ACTION_IDENTIFIER = 2

            self.action_topic_sub = None
            self.all_notifs_succeeded = True

            # Get node params
            self.robot_name = rospy.get_param('~robot_name', "my_gen3")

            rospy.loginfo("Using robot_name " + self.robot_name)

            # Init the action topic subscriber
            self.action_topic_sub = rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification, self.cb_action_topic)
            self.last_action_notif_type = None

            # Init the services
            clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
            rospy.wait_for_service(clear_faults_full_name)
            self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

            read_action_full_name = '/' + self.robot_name + '/base/read_action'
            rospy.wait_for_service(read_action_full_name)
            self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

            execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
            rospy.wait_for_service(execute_action_full_name)
            self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

            set_cartesian_reference_frame_full_name = '/' + self.robot_name + '/control_config/set_cartesian_reference_frame'
            rospy.wait_for_service(set_cartesian_reference_frame_full_name)
            self.set_cartesian_reference_frame = rospy.ServiceProxy(set_cartesian_reference_frame_full_name, SetCartesianReferenceFrame)

            activate_publishing_of_action_notification_full_name = '/' + self.robot_name + '/base/activate_publishing_of_action_topic'
            rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
            self.activate_publishing_of_action_notification = rospy.ServiceProxy(activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)
        except:
            self.is_init_success = False
        else:
            self.is_init_success = True

    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event

    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if (self.last_action_notif_type == ActionEvent.ACTION_END):
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
                rospy.loginfo("Received ACTION_ABORT notification")
                self.all_notifs_succeeded = False
                return False
            else:
                time.sleep(0.01)

    def example_clear_faults(self):
        try:
            self.clear_faults()
        except rospy.ServiceException:
            rospy.logerr("Failed to call ClearFaults")
            return False
        else:
            rospy.loginfo("Cleared the faults successfully")
            rospy.sleep(2.5)
            return True

    def example_home_the_robot(self):
        # The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
        req = ReadActionRequest()
        req.input.identifier = self.HOME_ACTION_IDENTIFIER
        self.last_action_notif_type = None
        try:
            res = self.read_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ReadAction")
            return False
        # Execute the HOME action if we could read it
        else:
            # What we just read is the input of the ExecuteAction service
            req = ExecuteActionRequest()
            req.input = res.output
            rospy.loginfo("Sending the robot home...")
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ExecuteAction")
                return False
            else:
                return self.wait_for_action_end_or_abort()

    def example_set_cartesian_reference_frame(self):
        # Prepare the request with the frame we want to set
        req = SetCartesianReferenceFrameRequest()
        req.input.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_MIXED

        # Call the service
        try:
            self.set_cartesian_reference_frame()
        except rospy.ServiceException:
            rospy.logerr("Failed to call SetCartesianReferenceFrame")
            return False
        else:
            rospy.loginfo("Set the cartesian reference frame successfully")
            return True

        # Wait a bit
        rospy.sleep(0.25)

    def example_subscribe_to_a_robot_notification(self):
        # Activate the publishing of the ActionNotification
        req = OnNotificationActionTopicRequest()
        rospy.loginfo("Activating the action notifications...")
        try:
            self.activate_publishing_of_action_notification(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call OnNotificationActionTopic")
            return False
        else:
            rospy.loginfo("Successfully activated the Action Notifications!")

        rospy.sleep(1.0)

        return True

    def joy_callback(self,joy_msg):
        self.joy_control = joy_msg.axes
        self.joy_buttons = joy_msg.buttons
        # rospy.loginfo(joy_control)

    def main(self):
        # For testing purposes
        rospy.Subscriber("joy", Joy, self.joy_callback)
        
        success = self.is_init_success
        try:
            rospy.delete_param("/kortex_examples_test_results/cartesian_poses_with_notifications_python")
        except:
            pass

        if success:

            #*******************************************************************************
            # Make sure to clear the robot's faults else it won't move if it's already in fault
            success &= self.example_clear_faults()
            #*******************************************************************************

            #*******************************************************************************
            # Subscribe to ActionNotification's from the robot to know when a cartesian pose is finished
            success &= self.example_subscribe_to_a_robot_notification()
            #*******************************************************************************
            
            #*******************************************************************************
            # Start the example from the Home position
            # success &= self.example_home_the_robot()
            #*******************************************************************************

            #*******************************************************************************
            # Set the reference frame to "Mixed"
            success &= self.example_set_cartesian_reference_frame()

            #*******************************************************************************
            # Prepare and send pose 1
            my_cartesian_speed = CartesianSpeed()
            my_cartesian_speed.translation = 0.5 # m/s
            my_cartesian_speed.orientation = 15  # deg/s
            my_constrained_pose = ConstrainedPose()
            my_constrained_pose.constraint.oneof_type.speed.append(my_cartesian_speed)

            while not rospy.is_shutdown():

                if self.joy_buttons[5]:
                    while  self.joy_buttons[5] != 0:
                        if self.joy_buttons[5] != 1:
                            break
                    rospy.loginfo("Arm and wrist control started")


                    while not rospy.is_shutdown():

                        if self.joy_buttons[5]:
                            while  self.joy_buttons[5] != 0:
                                if self.joy_buttons[5] != 1:
                                    break
                            rospy.loginfo("Arm and wrist control terminated")
                            break
                        #  Get current pose 
                        feedback = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)
                        my_constrained_pose.target_pose.x = feedback.base.commanded_tool_pose_x
                        my_constrained_pose.target_pose.y = feedback.base.commanded_tool_pose_y
                        my_constrained_pose.target_pose.z = feedback.base.commanded_tool_pose_z
                        my_constrained_pose.target_pose.theta_x = feedback.base.commanded_tool_pose_theta_x
                        my_constrained_pose.target_pose.theta_y = feedback.base.commanded_tool_pose_theta_y
                        my_constrained_pose.target_pose.theta_z = feedback.base.commanded_tool_pose_theta_z

                        #  Arm control
                        if self.joy_buttons[0]: # when button A is pressed
                            while  self.joy_buttons[0] != 0:
                                if self.joy_buttons[0] != 1:
                                    break
                            rospy.loginfo("Arm control started")
                            while not rospy.is_shutdown():
                                if self.joy_buttons[5]: # x jostick button to exit arm control
                                    while  self.joy_buttons[5] != 0:
                                        if self.joy_buttons[5] != 1:
                                            break
                                    rospy.loginfo("Arm control has terminated")
                                    break
                                
                                my_constrained_pose.target_pose.x += self.joy_control[1]*0.05
                                my_constrained_pose.target_pose.y += self.joy_control[0]*0.05
                                my_constrained_pose.target_pose.z += self.joy_control[7]*0.005   
                                
                                req = ExecuteActionRequest()
                                req.input.oneof_action_parameters.reach_pose.append(my_constrained_pose)
                                
                                req.input.name = "pose1"
                                req.input.handle.action_type = ActionType.REACH_POSE
                                req.input.handle.identifier = 1001
                                self.last_action_notif_type = None
                                try:
                                    self.execute_action(req)
                                except rospy.ServiceException:
                                    rospy.logerr("Failed to send pose")
                                    success = False
                                else:
                                    rospy.loginfo("Waiting for pose to finish...")

                                self.wait_for_action_end_or_abort()
                                success &= self.all_notifs_succeeded
                                rospy.sleep(0.1)

                        #  Wrist control
                        if self.joy_buttons[1]:  # When button B is pressed
                            while  self.joy_buttons[1] != 0:
                                if self.joy_buttons[1] != 1:
                                    break
                            rospy.loginfo("Wrist control started")
                            while not rospy.is_shutdown():
                                if self.joy_buttons[5]: # Jostick button to exit arm control
                                    while  self.joy_buttons[5] != 0:
                                        if self.joy_buttons[5] != 1:
                                            break
                                    rospy.loginfo("Wrist control has terminated")
                                    break
                                
                                my_constrained_pose.target_pose.theta_x += self.joy_control[1]*30
                                my_constrained_pose.target_pose.theta_y += self.joy_control[0]*20
                                my_constrained_pose.target_pose.theta_z += self.joy_control[6]*10
                                
                                req = ExecuteActionRequest()
                                req.input.oneof_action_parameters.reach_pose.append(my_constrained_pose)
                                req.input.name = "pose1"
                                req.input.handle.action_type = ActionType.REACH_POSE
                                req.input.handle.identifier = 1001
                                self.last_action_notif_type = None
                                try:
                                    self.execute_action(req)
                                except rospy.ServiceException:
                                    rospy.logerr("Failed to send pose")
                                    success = False
                                else:
                                    rospy.loginfo("Waiting for pose to finish...")

                                self.wait_for_action_end_or_abort()
                                success &= self.all_notifs_succeeded
                                rospy.sleep(0.1)

                        if self.joy_buttons[6]:  # When button B is pressed 
                                self.example_home_the_robot()
                                rospy.loginfo("Robot in home position")

        if not success:
            rospy.logerr("The arm and wrist control encountered an error.")

if __name__ == "__main__":
    ex = ExampleCartesianActionsWithNotifications()
    ex.main()
