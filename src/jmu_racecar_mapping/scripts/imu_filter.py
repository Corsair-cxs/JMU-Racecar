#!/usr/bin/env python

'''
This script makes Gazebo less fail by translating gazebo status messages to odometry data.
Since Gazebo also publishes data faster than normal odom data, this script caps the update to 20hz.
Winter Guerra
'''

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
from std_msgs.msg import Header

class ImuNode:
    # Set publishers
    # pub_imu = rospy.Publisher('/jmu_racecar_gazebo/odom', Odometry, queue_size=1)
    pub_imu = rospy.Publisher('/imu_filtered', Imu, queue_size=10)

    def __init__(self):
        # init internals
        last_received_Imu = Imu()
        self.last_recieved_stamp = None
        self.last_stamp = None
        self.kFakeGravity = 9.8

        # Set the update rate
        rospy.Timer(rospy.Duration(.01), self.timer_callback) # 200hz

        # Set subscribers
        rospy.Subscriber('/imu_data', Imu, self.sub_imu_data_update)

    def sub_imu_data_update(self, msg):
        # Find the index of the racecar
        try:
            self.last_received_Imu = msg
        except ValueError as e:
            # Wait for Gazebo to startup
            pass
        self.last_recieved_stamp = rospy.Time.now()

    def timer_callback(self, event):
        if self.last_recieved_stamp is None:
            return
        if self.last_recieved_stamp == self.last_stamp:
            return
        self.last_stamp = self.last_recieved_stamp
        #TODO
        # self.last_received_Imu.header.stamp = self.last_recieved_stamp
        self.last_received_Imu.linear_acceleration.z = self.kFakeGravity
        # I set this to be 'odom'
        # TODO
        # print(self.last_received_Imu)
        self.pub_imu.publish(self.last_received_Imu)

# Start the node
if __name__ == '__main__':
    rospy.init_node("imu_filter_node")
    node = ImuNode()
    rospy.spin()
