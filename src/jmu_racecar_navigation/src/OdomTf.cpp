#include "jmu_odom_tf/odom_tf.h"
using namespace std;

OdomTf::OdomTf(ros::NodeHandle* nodehandle) : nh_(*nodehandle) { 
    ROS_INFO("in class constructor of DemoTfListener");
    tfListener_ = new tf::TransformListener; 
    odom_tf_ready_ = false;
    odom_ready_ = false;
    amcl_ready_ = false;
    bool tferr = true;
    pose_stamped_publisher_= nh_.advertise<geometry_msgs::PoseStamped>("triad_display_pose", 1, true);
    amcl_est_base_pub_ = nh_.advertise<nav_msgs::Odometry>("amcl_est_odom", 1);
    odom_client = nh_.serviceClient<robotnik_msgs::set_odometry>("set_odometry");
    ROS_INFO("waiting for tf between base_footprint and odom...");
    while (tferr) {
        tferr = false;
        try {
            tfListener_->lookupTransform("odom", "base_footprint", ros::Time(0), stfBaseFootprintWrtOdom_); //stfBaseFootprintWrtOdom_后面没有用到
            ROS_INFO("odom ==> base_footprint tf is good"); 
        } catch (tf::TransformException &exception) {
            ROS_WARN("%s; retrying...", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }
    tferr = true;
    ROS_INFO("waiting for tf between odom and map...");
    while (tferr) {
        tferr = false;
        try {
            tfListener_->lookupTransform("map", "odom", ros::Time(0), stfOdomWrtMap_);  //stfOdomWrtMap_后面也没用到
            ROS_INFO("map ==> odom tf is good");
        } catch (tf::TransformException &exception) {
            ROS_WARN("%s; retrying...", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }
    
    stfDriftyOdomWrtMap_.setIdentity();  //StampedTransform
    stfDriftyOdomWrtMap_.stamp_ = ros::Time::now();           
    stfDriftyOdomWrtMap_.frame_id_ = "map";
    stfDriftyOdomWrtMap_.child_frame_id_ = "drifty_odom";
    tf::Quaternion quat(0, 0, 0, 1);
    stfDriftyOdomWrtMap_.setRotation(quat);  



    odom_subscriber_ = nh_.subscribe("/racecar_wheel_imu_odometry/odom", 1, &OdomTf::odomCallback, this); 
    ROS_WARN("waiting for drifty_odom publication");
    odom_count_=0;
    ROS_INFO("waiting for valid odom message...");
    while (!odom_ready_) {
        ros::Duration(0.005).sleep();
        ros::spinOnce();
    }
    ROS_INFO("Sleep over. Got odom callback ;   DriftyOdom_ --->  Base_Footprint ");

    //执行回调函数    
    amcl_subscriber_ = nh_.subscribe("/amcl_pose", 1, &OdomTf::amclCallback, this); 
    ROS_WARN("waiting for amcl publication...");
    while (!amcl_ready_) {
        ros::Duration(0.02).sleep();
        ros::spinOnce();
    }        
    ROS_INFO("Sleep over. Got amcl callback ;   Map_    --->   BaseFootprint:");
    ROS_INFO("constructor: got an odom message; ready to roll");
    odom_tf_ready_=true;
}

void OdomTf::initializeSubscribers() {
    ROS_INFO("Initializing Subscribers");
    odom_subscriber_ = nh_.subscribe("/racecar_wheel_imu_odometry/odom", 1, &OdomTf::odomCallback, this); 
    amcl_subscriber_ = nh_.subscribe("/amcl_pose", 1, &OdomTf::amclCallback, this); 
}
void OdomTf::odomCallback(const nav_msgs::Odometry& odom_rcvd) {
    odom_count_++;
    current_odom_ = odom_rcvd; 
    odom_pose_ = odom_rcvd.pose.pose;
    odom_x_ = odom_rcvd.pose.pose.position.x;
    odom_y_ = odom_rcvd.pose.pose.position.y;
    odom_quat_ = odom_rcvd.pose.pose.orientation;
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
    stfBaseFootprintWrtDriftyOdom_.frame_id_ = "drifty_odom";
    stfBaseFootprintWrtDriftyOdom_.child_frame_id_ = "amcl_base_footprint";
    stfBaseFootprintWrtDriftyOdom_.setOrigin(pos);
    stfBaseFootprintWrtDriftyOdom_.setRotation(q);

    stfDriftyOdomWrtBase_ = xform_utils.stamped_transform_inverse(stfBaseFootprintWrtDriftyOdom_); 
    xform_utils.multiply_stamped_tfs(stfDriftyOdomWrtMap_,stfBaseFootprintWrtDriftyOdom_,stfEstBaseWrtMap_);
    stfEstBaseWrtMap_.child_frame_id_ = "amcl_est_base";
    br_.sendTransform(stfEstBaseWrtMap_);
    estBasePoseWrtMap_ = xform_utils.get_pose_from_stamped_tf(stfEstBaseWrtMap_);
    pose_stamped_publisher_.publish(estBasePoseWrtMap_);  
    
    amcl_est_odom_ = odom_rcvd;
    amcl_est_odom_.header.stamp = ros::Time::now();
    amcl_est_odom_.header.frame_id = "odom";
    amcl_est_odom_.child_frame_id = "base_footprint";
    amcl_est_odom_.pose.pose.position.x = estBasePoseWrtMap_.pose.position.x;
    amcl_est_odom_.pose.pose.position.y = estBasePoseWrtMap_.pose.position.y;
    amcl_est_odom_.pose.pose.position.z = estBasePoseWrtMap_.pose.position.z;
    
    amcl_est_base_pub_.publish(amcl_est_odom_);  
    odom_ready_ = true;
}

void OdomTf::amclCallback(const geometry_msgs::PoseWithCovarianceStamped& amcl_rcvd) {
    amcl_pose_ = amcl_rcvd.pose.pose;
    amcl_quat_ = amcl_pose_.orientation;
    ROS_WARN("amcl pose: x, y, yaw: %f, %f, %f", amcl_pose_.position.x, amcl_pose_.position.y, xform_utils.convertPlanarQuat2Phi(amcl_pose_.orientation));
    tf::Vector3 pos;
    tf::Quaternion q;
    pos.setX(amcl_pose_.position.x);
    pos.setY(amcl_pose_.position.y);
    pos.setZ(amcl_pose_.position.z);
    q.setX(amcl_quat_.x);
    q.setY(amcl_quat_.y);
    q.setZ(amcl_quat_.z);
    q.setW(amcl_quat_.w);
    stfAmclBaseFootprintWrtMap_.stamp_ = ros::Time::now();
    stfAmclBaseFootprintWrtMap_.frame_id_ = "map";
    stfAmclBaseFootprintWrtMap_.child_frame_id_ = "amcl_base_footprint";
    stfAmclBaseFootprintWrtMap_.setOrigin(pos);
    stfAmclBaseFootprintWrtMap_.setRotation(q); 
    stfDriftyOdomWrtBase_ = xform_utils.stamped_transform_inverse(stfBaseFootprintWrtDriftyOdom_); 
    if (!xform_utils.multiply_stamped_tfs(stfAmclBaseFootprintWrtMap_,stfDriftyOdomWrtBase_,stfDriftyOdomWrtMap_)) {
        ROS_WARN("stfAmclBaseFootprintWrtMap_,stfDriftyOdomWrtBase_ multiply invalid" );
    }
    amcl_ready_=true;
}
