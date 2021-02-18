#ifndef ODOM_TF_H_
#define ODOM_TF_H_

#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <iostream>

#include <ros/ros.h> 
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

#include <robotnik_msgs/set_odometry.h>

#include <std_msgs/Float64.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/QuadWord.h>
#include <tf/transform_broadcaster.h>
#include <xform_utils/xform_utils.h>

class pf_OdomTf
{
public:
    pf_OdomTf(ros::NodeHandle* nodehandle);
    XformUtils xform_utils;
    tf::StampedTransform put_pose_in_transform(geometry_msgs::Pose pose);
    tf::StampedTransform get_tfBaseFootprintWrtDriftyOdom() { return stfBaseFootprintWrtDriftyOdom_; } 
   
    tf::StampedTransform stfBaseFootprintWrtOdom_; 
    tf::StampedTransform stfPfBaseFootprintWrtMap_; 
    tf::StampedTransform stfBaseFootprint_wrt_Map_; 
    tf::StampedTransform stfEstBaseWrtMap_;
    tf::StampedTransform tfLink2ToOdom_;  
    tf::StampedTransform stfBaseFootprintWrtDriftyOdom_;
    tf::StampedTransform stfDriftyOdomWrtBase_;
    tf::StampedTransform stfOdomWrtMap_; 
    tf::StampedTransform stfDriftyOdomWrtMap_;
    tf::StampedTransform stfMapWrtOdom_;
    tf::StampedTransform stfBase_FootprintWrtCorrectOdom_;
    
    geometry_msgs::PoseStamped base_link_wrt_odom_; 
    geometry_msgs::PoseStamped base_link_wrt_map_; 

    tf::TransformListener* tfListener_;   
    tf::TransformBroadcaster br_;
    bool odom_tf_ready_;
    bool odom_tf_is_ready() { return odom_tf_ready_; }
    bool pf_odom_ready_;

private:
    ros::NodeHandle nh_; 
    ros::Subscriber odom_subscriber_; 
    void initializeSubscribers();
    void odomCallback(const nav_msgs::Odometry& odom_rcvd);
    ros::Publisher pose_publisher_; 
    ros::ServiceClient odom_client;

    robotnik_msgs::set_odometry odom_srv;

    geometry_msgs::PoseStamped estOdomWrtMap_;

    nav_msgs::Odometry current_odom_;
    geometry_msgs::Pose odom_pose_;   
    int odom_count_;
    double odom_x_;
    double odom_y_;
    double odom_phi_;
    geometry_msgs::Quaternion odom_quat_; 
}; 

#endif  
