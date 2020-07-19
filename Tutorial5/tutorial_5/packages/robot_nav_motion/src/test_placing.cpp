#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include<std_srvs/Empty.h>
// MoveIt! headers
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_listener.h>
// Std C++ headers
#include <string>
#include <vector>
#include <map>
#include <robot_nav_motion/joint.h>
//#include <from2Dto3D/PointBase.h>
using namespace tiago;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_arm");
  ros::NodeHandle n;
  ros::Rate r(60);

 
     
  /******************************************************************************************************************************************/

  /*Moving the robot arm to a pre-position for placing

  /******************************************************************************************************************************************/
    ROS_INFO("C ready for placing");
    
    tiago::navigation(4.04103660583,-1.28337025642,0.0,0.0,-0.791229184193,0.611519728285);
    
    ROS_INFO("C for placing");
     
     tiago::jointlift lifter;
     double pose_1[8]={0.15, 0.2, -1.34, -0.2, 1.94, -1.57, 0.0, 0.0};
     lifter.action(pose_1);
     pose_1[0]= 0.19;
     lifter.action(pose_1);
     pose_1[2]= -0.21;
     pose_1[3]= -1.46;
     pose_1[4]= 2.08;
     lifter.action(pose_1);
     pose_1[5]= -1.73;
     pose_1[6]= -0.94;
     pose_1[7]= -1.68;
     pose_1[1]= 0.44;
     lifter.action(pose_1);
     
     
    //move forward to place the obeject
     double pose_2[8]={0.19, 0.44, -0.21, -1.54, 2.08, -1.73, -0.94, -1.68};    
     lifter.action(pose_2);
     pose_2[0]=0.24;
     lifter.action(pose_2);
     pose_2[1]=1.08;
     pose_2[4]= 1.10;
     pose_2[6]= -0.44;
     pose_2[7]= -1.58;
     lifter.action(pose_2);  
     //release the gripper
     system("rosrun pal_gripper_controller_configuration_gazebo home_gripper.py");

     //lift the torso for rotate
     double pose_3[8]={0.24, 0.44, -0.21, -1.54, 2.08, -1.73, -0.94, -1.68};    
     lifter.action(pose_3);
     pose_3[0]=0.35;
     lifter.action(pose_3);
     



      
    /******************************************************************************************************************************************/

    /*Moving back a certain distance B for Home pose (90 degree rotation)

    /******************************************************************************************************************************************/
    tiago::pub=n.advertise< geometry_msgs::Twist>("/mobile_base_controller/cmd_vel",100);
      ros::Time ti, tf;
      ti=ros::Time::now();
       tf=ti;
       geometry_msgs::Twist msg;
       while((tf.toSec()-ti.toSec())<=4)
     {   
    
       tf=ros::Time::now();
       msg.angular.z=0.5;//make robot self rotate
       pub.publish(msg);
       ros::spinOnce();
     r.sleep();
     }
    
    //tigao::navigation(4.08103660583,-1.30337025642,0.0,0.0, 0.009548555123, 0.999954411508);
    ROS_INFO("D for homing");
    system("rosrun tiago_gazebo tuck_arm.py ");
     
     ros::spin();
     return 0;
}
