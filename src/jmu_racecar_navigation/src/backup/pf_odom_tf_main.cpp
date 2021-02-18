//odom_tf.cpp:
//wsn, March 2016
//illustrative node to show use of tf listener to reconcile odom and map frames
// w/rt base_link

// this header incorporates all the necessary #include files and defines the class "OdomTf"
#include "jmu_pf_odom_tf/pf_odom_tf.h"

using namespace std;

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "pf_OdomTf_node"); 
    ros::NodeHandle nh; 
    ROS_INFO("main: instantiating an object of type pf_OdomTf");
    pf_OdomTf pf_OdomTf(&nh); 
    ROS_INFO:("starting main loop");
    ros::Rate sleep_timer(50.0);
    while (ros::ok()) {
        ros::spinOnce();
        sleep_timer.sleep(); 
    }
    return 0;
} 

