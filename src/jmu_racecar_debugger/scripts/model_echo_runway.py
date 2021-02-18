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
from gazebo_msgs.msg import LinkStates, ModelStates
import tf 

class Msg_print_Node:
    def __init__(self):
        rospy.Subscriber('/racecar/ackermann_cmd',AckermannDriveStamped,self.ackermann_cmd_cb)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.sub_robot_pose_update)

    def ackermann_cmd_cb(self,ackermann_cmd):    
        # print ackermann_cmd.drive
        # print("-------------")
        return

    def sub_robot_pose_update(self, msg):
        # Find the index of the racecar
        try: 
            arrayIndex = msg.name.index('jmu_racecar')
        except ValueError as e:
            # Wait for Gazebo to startup
            pass
        else:
            self.last_received_pose = msg.pose[arrayIndex]
            self.last_received_twist = msg.twist[arrayIndex]
            self.quaternion_ = self.last_received_pose.orientation
            point = tf.transformations.euler_from_quaternion((self.quaternion_.x,self.quaternion_.y,self.quaternion_.z,self.quaternion_.w)) 
        print self.last_received_pose
        # print self.quaternion_
        print point
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
