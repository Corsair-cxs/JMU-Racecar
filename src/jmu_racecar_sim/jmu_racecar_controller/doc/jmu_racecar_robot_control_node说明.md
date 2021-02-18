-- jmu_racecar_robot_control_node.cpp
1. 设置线速度和角速度的限制值
    void setCommand(const ackermann_msgs::AckermannDriveStamped &msg)
    {   
        // Mapping - linear = v_ref_, angular = alfa_ref_ 
        double speed_limit = 10.0;  // m/s
        double angle_limit = PI/4.0;   // there should be also urdf limits 舵机打角最大值
        v_ref_ = saturation(msg.drive.speed, -speed_limit, speed_limit);  
        alfa_ref_ = saturation(msg.drive.steering_angle, -angle_limit, angle_limit);
    }
参数:
    v_ref_ 和 alfa_ref_ 分别是取限制范围内的线速度和角速度值.

2. 获得元素在消息队列中的下标值
    /// Controller startup in realtime
    int starting()
    {
    // Initialize joint indexes according to joint names 
    if (read_state_) {
            vector<string> joint_names = joint_state_.name;
            frw_vel_ = find (joint_names.begin(),joint_names.end(), string(joint_front_right_wheel)) - joint_names.begin();
            flw_vel_ = find (joint_names.begin(),joint_names.end(), string(joint_front_left_wheel)) - joint_names.begin();
            blw_vel_ = find (joint_names.begin(),joint_names.end(), string(joint_back_left_wheel)) - joint_names.begin();
            brw_vel_ = find (joint_names.begin(),joint_names.end(), string(joint_back_right_wheel)) - joint_names.begin();
            frw_pos_ = find (joint_names.begin(),joint_names.end(), string(joint_front_right_steer)) - joint_names.begin();
            flw_pos_ = find (joint_names.begin(),joint_names.end(), string(joint_front_left_steer)) - joint_names.begin();
        return 0;
    }else{
            ROS_WARN("racecarControllerClass::starting: joint_states are not being received");
            return -1;
    }
参数:
    frw_vel_ ; flw_vel_ ; blw_vel_ ; brw_vel_ ; frw_pos_ ; flw_pos_