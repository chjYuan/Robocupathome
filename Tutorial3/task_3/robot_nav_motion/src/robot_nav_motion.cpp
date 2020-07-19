#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include<std_srvs/Empty.h>
// MoveIt! headers
#include <moveit/move_group_interface/move_group_interface.h>

// Std C++ headers
#include <string>
#include <vector>
#include <map>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_nav_motion");
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
  
  /******************************************************************************************************************************************/

  /*FIrst move from A to C and flip the table/

  /******************************************************************************************************************************************/
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
    ROS_INFO("The Tiago failed to move  for some reason");
  
  
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
    ROS_INFO("The Tiago failed to move for some reason");

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
    ROS_INFO("The Tiago failed to move  for some reason");

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
    ROS_INFO("The Tiago failed to move for some reason");

  // set goal as 2D coordinate for C for lifting the table
  goal.target_pose.pose.position.x = -0.807262295485;
  goal.target_pose.pose.position.y = -12.0203562927;
  //set goal orientation
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = 0.905360046198;
  goal.target_pose.pose.orientation.w = 0.908849211911;
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO(" Tiago moved to C");
  else
    ROS_INFO("The Tiago failed to move for some reason");
  
  
  //lift the table using planning in cartesian space, follwing is the goal position of arm_tool_link
  geometry_msgs::PoseStamped goal_pose;
  goal_pose.header.frame_id = "base_footprint";
  goal_pose.pose.position.x = 0.68868;//
  goal_pose.pose.position.y = 0.23033;
  goal_pose.pose.position.z = 0.8467;
  // goal_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(-0.011, 1.57, 0.037);
  goal_pose.pose.orientation.x=0.11064;
  goal_pose.pose.orientation.y=0.01438;
  goal_pose.pose.orientation.z=0.144619;
  goal_pose.pose.orientation.w=0.783177;
 
  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::vector<std::string> torso_arm_joint_names;
  //select group of joints
  moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");
  //choose your preferred planner
  group_arm_torso.setPlannerId("SBLkConfigDefault");
  group_arm_torso.setPoseReferenceFrame("base_footprint");
  group_arm_torso.setPoseTarget(goal_pose);

  ROS_INFO_STREAM("Planning to move " <<
                  group_arm_torso.getEndEffectorLink() << " to a target pose expressed in " <<
                  group_arm_torso.getPlanningFrame());

  group_arm_torso.setStartStateToCurrentState();
  group_arm_torso.setMaxVelocityScalingFactor(1.0);


  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  //set maximum time to find a plan
  group_arm_torso.setPlanningTime(5.0);
  bool success =bool(group_arm_torso.plan(my_plan));

  if ( !success )
    throw std::runtime_error("No plan found");

  ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

  // Execute the plan

  group_arm_torso.move();
  //go back to the home position
  system("rosrun tiago_gazebo tuck_arm.py");
  spinner.stop();
  /******************************************************************************************************************************************/
  
  /*When finishing bringing the arm to the home position, tiago start patroling
  
  /******************************************************************************************************************************************/
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
    ROS_INFO("The Tiago failed to move  for some reason");

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
    ROS_INFO("The Tiago failed to move for some reason");

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
    ROS_INFO("The Tiago failed to move  for some reason");

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
    ROS_INFO("The Tiago failed to move for some reason");

  // set goal as 2D coordinate for C for lifting the table
  goal.target_pose.pose.position.x = -0.807262295485;
  goal.target_pose.pose.position.y = -12.0203562927;
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
    ROS_INFO("The Tiago failed to move for some reason");
  
  
   }
  

  return 0;
}