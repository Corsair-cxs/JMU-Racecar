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

class Msg_print_Node:
    def __init__(self):
        rospy.Subscriber('/racecar/ackermann_cmd',AckermannDriveStamped,self.ackermann_cmd_cb)
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.sub_robot_pose_update)

    def ackermann_cmd_cb(self,ackermann_cmd):    
        # print ackermann_cmd.drive
        # print("-------------")
        return

    def sub_robot_pose_update(self, msg):
        # Find the index of the racecar
        try: 
            arrayIndex = msg.name.index('jmu_racecar::base_footprint')
        except ValueError as e:
            # Wait for Gazebo to startup
            pass
        else:
            # Extract our current position information
            self.last_received_pose = msg.pose[arrayIndex]
            # self.last_received_pose.position.x += 0.5   #use_racetrack
            self.last_received_pose.position.z -= 0.2   #use_warehouse
            self.last_received_twist = msg.twist[arrayIndex]
        print self.last_received_pose
        print("-------------")
        # print self.last_received_twist
        # print("-------------")

    os.system("clear")

# Start the node
if __name__ == '__main__':
    rospy.init_node('msg_echo_Node')
    node = Msg_print_Node()
    rospy.spin()


    # # Set subscribers
