   rostopic echo /cloud | grep frame_id

   1.  rostopic type /gazebo/link_states
         gazebo_msgs/LinkStates
   2.  rosmsg show gazebo_msgs/LinkStates
         string[] name
         geometry_msgs/Pose[] pose
            geometry_msgs/Point position
               float64 x
               float64 y
               float64 z
            geometry_msgs/Quaternion orientation
               float64 x
               float64 y
               float64 z
               float64 w
         geometry_msgs/Twist[] twist
            geometry_msgs/Vector3 linear
               float64 x
               float64 y
               float64 z
            geometry_msgs/Vector3 angular
               float64 x
               float64 y
               float64 z
            
   3.  rostopic echo /gazebo/link_states/name
   4.  rostopic echo /gazebo/link_states | grep name
   5.  rostopic echo /gazebo/link_states | grep pose
