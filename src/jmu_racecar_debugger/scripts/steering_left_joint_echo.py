#!/usr/bin/env python

'''
This script makes Gazebo less fail by translating gazebo status messages to odometry data.
Since Gazebo also publishes data faster than normal odom data, this script caps the update to 20hz.
Winter Guerra
'''
import os
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64
from controller_manager_msgs.srv import ListControllers
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Transform, TransformStamped
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState

class Msg_print_Node:
    def __init__(self):
        rospy.Subscriber('/racecar/joint_states',JointState,self.sub_robot_joint_update)

    def sub_robot_joint_update(self, msg):
        # Find the index of the racecar
        try: 
            arrayIndex = msg.name.index('left_steering_joint')
        except ValueError as e:
            # Wait for Gazebo to startup
            pass
        else:
            self.last_received_joint = msg.position[msg.name.index('left_steering_joint')]
            # self.last_received_joint = msg.position[msg.name.index('right_steering_joint')]
            # self.last_received_joint = msg.name
        print self.last_received_joint
        print("-------------")
        # print self.last_received_twist
        # print("-------------")

    os.system("clear")

# Start the node
if __name__ == '__main__':
    rospy.init_node('steering_left_joint_msg_echo_Node')
    node = Msg_print_Node()
    rospy.spin()


    # # Set subscribers
