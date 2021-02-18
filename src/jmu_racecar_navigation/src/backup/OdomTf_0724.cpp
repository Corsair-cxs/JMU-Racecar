#include "jmu_odom_tf/odom_tf.h"
using namespace std;

OdomTf::OdomTf(ros::NodeHandle* nodehandle) : nh_(*nodehandle) { 
    ROS_INFO("in class constructor of DemoTfListener");
    tfListener_ = new tf::TransformListener; 
    odom_tf_ready_ = false;
    odom_ready_ = false;
    amcl_ready_ = false;
    bool tferr = true;
    pose_publisher_= nh_.advertise<geometry_msgs::PoseStamped>("triad_display_pose", 1, true);
    odom_client = nh_.serviceClient<robotnik_msgs::set_odometry>("set_odometry");
//============================print
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
//============================print_end

    //TODO map -> base_footprint     amclCallback
    stfAmclBaseFootprintWrtMap_.setIdentity();  //StampedTransform
    stfAmclBaseFootprintWrtMap_.stamp_ = ros::Time::now();           
    stfAmclBaseFootprintWrtMap_.frame_id_ = "map";
    stfAmclBaseFootprintWrtMap_.child_frame_id_ = "amcl_base_footprint";
    tf::Quaternion quat(0, 0, 0, 1);
    stfAmclBaseFootprintWrtMap_.setRotation(quat);  

    //TODO map -> odom       amclCallback odomCallback
    stfDriftyOdomWrtMap_ = stfAmclBaseFootprintWrtMap_;
    stfDriftyOdomWrtMap_.frame_id_ = "map"; 
    stfDriftyOdomWrtMap_.child_frame_id_ = "drifty_odom"; 

    
    //执行回调函数
    odom_subscriber_ = nh_.subscribe("/racecar_wheel_imu_odometry/odom", 1, &OdomTf::odomCallback, this); 
    ROS_WARN("waiting for drifty_odom publication");
    odom_count_=0;
    //TODO 延时可能要调整
    odom_phi_ = 1000.0;     // put in impossible value for heading; test this value to make sure we have received a viable odom message
    ROS_INFO("waiting for valid odom message...");
    while (!odom_ready_) {
        ros::Duration(0.01).sleep(); // sleep for half a second
        ros::spinOnce();
    }
    ROS_INFO("Sleep over. Got odom callback ;   DriftyOdom_ --->  Base_Footprint ");
    

    //执行回调函数    
    amcl_subscriber_ = nh_.subscribe("/amcl_pose", 1, &OdomTf::amclCallback, this); 
    ROS_WARN("waiting for amcl publication...");
    while (!amcl_ready_) {
        ros::Duration(0.01).sleep();
        ros::spinOnce();
    }        
    ROS_INFO("Sleep over. Got amcl callback ;   Map_    --->   BaseFootprint:");
    //TODO
    // xform_utils.printStampedTf(stfAmclBaseFootprintWrtMap_);
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
    
    current_odom_ = odom_rcvd; // nav_msgs::Odometry current_odom_;
    odom_pose_ = odom_rcvd.pose.pose;   // geometry_msgs::Pose odom_pose_;  
    //odom_vel_ = odom_rcvd.twist.twist.linear.x;
    //odom_omega_ = odom_rcvd.twist.twist.angular.z;
    odom_x_ = odom_rcvd.pose.pose.position.x;
    odom_y_ = odom_rcvd.pose.pose.position.y;
    odom_quat_ = odom_rcvd.pose.pose.orientation;
    odom_phi_ = xform_utils.convertPlanarQuat2Phi(odom_quat_);

    //values
    tf::Vector3 pos;
    tf::Quaternion q;
    pos.setX(odom_pose_.position.x);
    pos.setY(odom_pose_.position.y);
    pos.setZ(odom_pose_.position.z);
    q.setX(odom_quat_.x);
    q.setY(odom_quat_.y);
    q.setZ(odom_quat_.z);
    q.setW(odom_quat_.w);

    //set values
    //tf::StampedTransform stfBaseFootprintWrtDriftyOdom_;
    stfBaseFootprintWrtDriftyOdom_.stamp_ = ros::Time::now();
    stfBaseFootprintWrtDriftyOdom_.frame_id_ = "drifty_odom";
    stfBaseFootprintWrtDriftyOdom_.child_frame_id_ = "amcl_base_footprint";
    stfBaseFootprintWrtDriftyOdom_.setOrigin(pos);
    stfBaseFootprintWrtDriftyOdom_.setRotation(q);
    //cout<<endl<<"odom_count: "<<odom_count_<<endl;


    //tf::StampedTransform stfDriftyOdomWrtBase_;
    //这里用了反转变换,将坐标树的父子关系转化
    //base_footprint-->odom
    stfDriftyOdomWrtBase_ = xform_utils.stamped_transform_inverse(stfBaseFootprintWrtDriftyOdom_); 
    //TODO
    // br_.sendTransform(stfDriftyOdomWrtBase_);

    //TODO 乘法结果: map--->drifty_base_footprint
/*
    输入Ａ：stfDriftyOdomWrtMap_： map --> drifty_odom
    输入Ｂ：stfBaseFootprintWrtDriftyOdom_: drifty_odom-->drifty_base_footprint
    输出Ｃ：map-->drifty_base_footprint
*/

    //TODO
    xform_utils.multiply_stamped_tfs(stfDriftyOdomWrtMap_,stfBaseFootprintWrtDriftyOdom_,stfEstBaseWrtMap_);
    stfEstBaseWrtMap_.child_frame_id_ = "amcl_est_base";
    //TODO
    br_.sendTransform(stfEstBaseWrtMap_);

    // geometry_msgs::PoseStamped estBasePoseWrtMap_;
    estBasePoseWrtMap_ = xform_utils.get_pose_from_stamped_tf(stfEstBaseWrtMap_);
    pose_publisher_.publish(estBasePoseWrtMap_);    // topic:triad_display_pose 这个准确性待验证(直接是map-->base_footprint)
    odom_ready_ = true;
}

void OdomTf::amclCallback(const geometry_msgs::PoseWithCovarianceStamped& amcl_rcvd) {
    amcl_pose_ = amcl_rcvd.pose.pose;
    amcl_quat_ = amcl_pose_.orientation;
    ROS_WARN("amcl pose: x, y, yaw: %f, %f, %f", amcl_pose_.position.x, amcl_pose_.position.y, xform_utils.convertPlanarQuat2Phi(amcl_pose_.orientation));

    tf::Vector3 pos;
    tf::Quaternion q;

    //从amcl_pose中获得的参数
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
    

    // stfDriftyOdomWrtBase_ = xform_utils.stamped_transform_inverse(stfBaseLinkWrtDriftyOdom_); 

    // tfListener_->lookupTransform("odom", "base_footprint", ros::Time(0), stfDriftyOdomWrtBase_);  

    //反转, amcl_base_footprint --> odom
    stfDriftyOdomWrtBase_ = xform_utils.stamped_transform_inverse(stfBaseFootprintWrtDriftyOdom_); 
    // TODO
    // stfDriftyOdomWrtBase_.frame_id_ = "amcl_base_footprint";
    //乘积:map->odom
    
/*
    输入Ａ：stfAmclBaseFootprintWrtMap_： map --> amcl_base_footprint
    输入Ｂ：stfDriftyOdomWrtBase_: amcl_base_footprint-->odom
    输出Ｃ：stfDriftyOdomWrtMap_ :map-->    odom
*/
    if (!xform_utils.multiply_stamped_tfs(stfAmclBaseFootprintWrtMap_,stfDriftyOdomWrtBase_,stfDriftyOdomWrtMap_)) {
        ROS_WARN("stfAmclBaseFootprintWrtMap_,stfDriftyOdomWrtBase_ multiply invalid" );
    }
    // stfDriftyOdomWrtMap_.child_frame_id_ = "odom";   
    // br_.sendTransform(stfDriftyOdomWrtMap_);   
    //TODO
    // stfDriftyOdomWrtMap_.child_frame_id_ = "drifty_odom"; 
    // stfAmclBaseFootprintWrtMap_.child_frame_id_ = "amcl_base_footprint";
    // br_.sendTransform(stfAmclBaseFootprintWrtMap_);

    amcl_ready_=true;
}
