#include "jmu_pf_odom_tf/pf_odom_tf.h"
using namespace std;

pf_OdomTf::pf_OdomTf(ros::NodeHandle* nodehandle) : nh_(*nodehandle) { 
    ROS_INFO("in class constructor of DemoTfListener");
    tfListener_ = new tf::TransformListener; 
    pf_odom_ready_ = false;
    bool tferr = true;
    pose_publisher_= nh_.advertise<geometry_msgs::PoseStamped>("display_map_odom", 1, true);
    ROS_INFO("waiting for tf between base_footprint and odom...");
    while (tferr) {
        tferr = false;
        try {
            tfListener_->lookupTransform("odom", "base_footprint", ros::Time(0), stfBaseFootprintWrtOdom_); 
            ROS_INFO("odom ==> base_footprint tf is good"); 
        } catch (tf::TransformException &exception) {
            ROS_WARN("%s; retrying...", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); 
            ros::spinOnce();
        }
    }
    tferr = true;
    ROS_INFO("waiting for tf between pf_base_footprint and map...");
    while (tferr) {
        tferr = false;
        try {
            tfListener_->lookupTransform("map", "pf_base_footprint", ros::Time(0), stfOdomWrtMap_); 
            ROS_INFO("map ==> pf_base_footprint tf is good");
        } catch (tf::TransformException &exception) {
            ROS_WARN("%s; retrying...", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); 
            ros::spinOnce();
        }
    }

    //执行回调函数
    odom_subscriber_ = nh_.subscribe("/racecar_wheel_imu_odometry/odom", 1, &pf_OdomTf::odomCallback, this); 
    ROS_WARN("waiting for odom publication");
    odom_count_=0;
    ROS_INFO("waiting for valid odom message...");
    while (!pf_odom_ready_) {
        ros::Duration(0.01).sleep(); // sleep for half a second
        ros::spinOnce();
    }
    ROS_INFO("Sleep over. Got odom callback ;map--->odom");
}

void pf_OdomTf::odomCallback(const nav_msgs::Odometry& odom_rcvd) {
    odom_count_++;
    cout<<endl<<"odom_count: "<<odom_count_<<endl;
    current_odom_ = odom_rcvd; 
    odom_pose_ = odom_rcvd.pose.pose;   
    odom_quat_ = odom_rcvd.pose.pose.orientation;
    odom_phi_ = xform_utils.convertPlanarQuat2Phi(odom_quat_);

    tf::Vector3 pos;
    tf::Quaternion q;

    pos.setX(odom_pose_.position.x);
    pos.setY(odom_pose_.position.y);
    pos.setZ(odom_pose_.position.z);

    q.setX(odom_quat_.x);
    q.setY(odom_quat_.y);
    q.setZ(odom_quat_.z);
    q.setW(odom_quat_.w);
    
    stfBaseFootprintWrtDriftyOdom_.stamp_ = ros::Time::now();
    stfBaseFootprintWrtDriftyOdom_.frame_id_ = "odom";
    stfBaseFootprintWrtDriftyOdom_.child_frame_id_ = "pf_base_footprint";
    stfBaseFootprintWrtDriftyOdom_.setOrigin(pos);
    stfBaseFootprintWrtDriftyOdom_.setRotation(q);
    
    stfDriftyOdomWrtBase_ = xform_utils.stamped_transform_inverse(stfBaseFootprintWrtDriftyOdom_); 

    tfListener_->lookupTransform("map", "pf_base_footprint", ros::Time(0), stfPfBaseFootprintWrtMap_); 
    

    xform_utils.multiply_stamped_tfs(stfPfBaseFootprintWrtMap_,stfDriftyOdomWrtBase_,stfDriftyOdomWrtMap_);

    br_.sendTransform(stfDriftyOdomWrtMap_);
    pose_publisher_.publish(stfDriftyOdomWrtMap_);
    pf_odom_ready_ = true;
}