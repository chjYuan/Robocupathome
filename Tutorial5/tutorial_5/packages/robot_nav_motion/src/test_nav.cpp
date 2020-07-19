#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include<std_srvs/Empty.h>
// MoveIt! headers
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_listener.h>
// Std C++ headers
#include <string>
#include <vector>
#include <map>
#include <robot_nav_motion/joint.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
using namespace tiago;

int main(int argc, char** argv){
  ros::init(argc, argv, "test_nav");
  ros::NodeHandle n;
  ros::Rate r(60);
  geometry_msgs::Twist msg;
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  ros::Time ti, tf;
  ti=ros::Time::now();
  tf=ti;
  

  /******************************************************************************************************************************************/

  /*Rotating for Localization

  /******************************************************************************************************************************************/
  ros::Publisher pub=n.advertise< geometry_msgs::Twist>("/mobile_base_controller/cmd_vel",100);
  
  double t_rotate;
  std_srvs::Empty srv_empty; 
  //wait for the services to come up
  ros::Duration(5.0).sleep();
  // locate the robot in the world
  //ros::service::call(" /pal_navigation_sm",'LOC');
  //ros::service::call("/pal_map_manager/change_map",'input: 'pal_room'');
  ros::service::call("/global_localization", srv_empty);
  
  while((tf.toSec()-ti.toSec())<=30)
  {   
    
     tf=ros::Time::now();
       msg.angular.z=1.0;//make robot self rotate
       pub.publish(msg);
     ros::spinOnce();
     r.sleep();
  }
  ros::service::call("/move_base/clear_costmaps", srv_empty);
  ROS_INFO("Localization finished");
  
   while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
 
  /*move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now(); 
  
  /******************************************************************************************************************************************/

  /*Moving the robot arm to a pre-position A for grasping

  /******************************************************************************************************************************************/
  //  set goal as 2D coordinate for A
  /*goal.target_pose.pose.position.x = 3.62560582161;
  goal.target_pose.pose.position.y = -0.871481537819;
  //set goal orientation
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = -0.808273690062;
  goal.target_pose.pose.orientation.w = 0.588806964932;
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO(" Tiago moved to A");
  else
    ROS_INFO("The Tiago failed to move  for some reason");*/
  tiago::navigation(2.77406978607,-0.993755102158,0.0,0.0,-0.747944095454,0.663761726884);//-0.747944095454 ,0.663761726884
  ROS_INFO("A for grasping");
  system("rosrun tiago_gazebo tuck_arm.py ");
  ROS_INFO("A for grasping");
  tiago::navigation(4.00103660583,-1.18337025642,0.0,0.0,-0.791229184193,0.611519728285);
  


      return 0;
}