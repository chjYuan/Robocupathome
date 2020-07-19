#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include<std_srvs/Empty.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_navigation");
  ros::NodeHandle n;
  ros::Rate r(60);
  geometry_msgs::Twist msg;
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  ros::Time ti, tf;
  ti=ros::Time::now();
  tf=ti;


  ros::Publisher pub=n.advertise< geometry_msgs::Twist>("/mobile_base_controller/cmd_vel",100);
  
  double t_rotate;
  std_srvs::Empty srv_empty; 
  //wait for the services to come up
  ros::Duration(5.0).sleep();
  // locate the robot in the world
  ros::service::call("/global_localization", srv_empty);
  
  while((tf.toSec()-ti.toSec())<=25)
  {   
    
     tf=ros::Time::now();
       msg.angular.z=1.0;//make robot self rotate
       pub.publish(msg);
     ros::spinOnce();
     r.sleep();
  }
  ros::service::call("/move_base/clear_costmaps", srv_empty);
  ROS_INFO("Localization finished");
  
  
  //wait for the action server to come up
   while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
 
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  
  while(ros::ok())
  {
  //  set goal as 2D coordinate for A-
  goal.target_pose.pose.position.x = -1.07883024216;
  goal.target_pose.pose.position.y = -0.214169025421;
  //set goal orientation
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = -0.857760062798;
  goal.target_pose.pose.orientation.w = 0.51405026473;
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO(" Tiago moved to A");
  else
    ROS_INFO("The Tiago failed to move forward 1 meter for some reason");
  
  
  
  
  //  set goal as 2D coordinate for A
  goal.target_pose.pose.position.x = -2.50333092213;
  goal.target_pose.pose.position.y = -2.3698515892;
  //set goal orientation
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = 0.995643897813;
  goal.target_pose.pose.orientation.w = 0.0932374857445;
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO(" Tiago moved to A");
  else
    ROS_INFO("The Tiago failed to move forward 1 meter for some reason");

  //  set goal as 2D coordinate for A+
  goal.target_pose.pose.position.x = -3.50932645798;
  goal.target_pose.pose.position.y = -0.815170049667;
  //set goal orientation
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = 0.308702930911;
  goal.target_pose.pose.orientation.w = 0.951158504376;
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO(" Tiago moved to A+");
  else
    ROS_INFO("The Tiago failed to move forward 1 meter for some reason");

  //set goal as 2D coordinate for B
  goal.target_pose.pose.position.x = -4.39750671387;
  goal.target_pose.pose.position.y = -7.02008008957;
  //set goal orientation
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = -0.708849211911;
  goal.target_pose.pose.orientation.w = 0.705360046198;
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO(" Tiago moved to B");
  else
    ROS_INFO("The Tiago failed to move forward 1 meter for some reason");

  //set goal as 2D coordinate for C
  goal.target_pose.pose.position.x = -0.807262295485;
  goal.target_pose.pose.position.y = -12.1503562927;
  //set goal orientation
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = -0.0126337243136;
  goal.target_pose.pose.orientation.w = 0.99992019132;
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO(" Tiago moved to C");
  else
    ROS_INFO("The Tiago failed to move forward 1 meter for some reason");
 
  }

  return 0;
}