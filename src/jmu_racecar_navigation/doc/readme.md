说明:如果需要修改里程计的方案,需要配置以下参数:

teb_local_planner_params.yaml:
  odom_topic: racecar_wheel_imu_odometry/wheel_imu_odom
  # odom_topic: odom

path_pursuit.py:
        self.current_pose = rospy.Subscriber('/racecar_wheel_imu_odometry/wheel_imu_odom', Odometry, self.callback_read_current_position, queue_size=1)
        # self.current_pose = rospy.Subscriber('/odom', Odometry, self.callback_read_current_position, queue_size=1)
