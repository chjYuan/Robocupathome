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

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using namespace tiago;


int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_arm");
  ros::NodeHandle n;
  ros::Rate r(60);


  tiago::jointlift lifter;
  tiago::pub=n.advertise< geometry_msgs::Twist>("/mobile_base_controller/cmd_vel",100);
  double pose[8]={0.35, 0.2, -1.34, -0.2, 1.94, -1.57, 0.0, 0.0};
  lifter.action(pose);
  
  /***********************************************************************************************************************************
  Wating for stable state to call service transform(get handle position) 
  ***********************************************************************************************************************************/

  ros::Time ti_0, tf_0;
    ti_0=ros::Time::now();
    tf_0=ti_0;
    geometry_msgs::Twist msg_0;
    while((tf_0.toSec()-ti_0.toSec())<=10)
  {   
    
      tf_0=ros::Time::now();
      ros::spinOnce();
      r.sleep();
  }
    ROS_INFO("A reaching");
    

  /******************************************************************************************************************************************/
  /*Get the current handle 3-D Position from depth Camera
  /******************************************************************************************************************************************/ 
  
  //launch handle_3d transform
  ros::ServiceClient client_0 = n.serviceClient<tf_tiago::ReachGoal>("reachCheck_handle");
  tf_tiago::ReachGoal srv_0;
  srv_0.request.req = 1;
  if (client_0.call(srv_0))
    {         
        ROS_INFO_STREAM("succeed to launch Transform");
    }
  else
    {
      ROS_ERROR("Failed to launch Transform");
      return 1;
    }   
    pose[0]=0.35;
    pose[1]= 0.26;
    pose[2]= 0.01;
    pose[3]= -1.58;
    //lifter.action(pose);
    pose[4]= 1.29;
    pose[5]= -1.44;
    pose[6]= 1.19;
    pose[7]=1.57;
    lifter.action(pose);
    
    //wait for values
    ros::Time ti_1, tf_1;
    ti_1=ros::Time::now();
    tf_1=ti_1;
    while((tf_1.toSec()-ti_1.toSec())<=30)
      {   
        
          tf_1=ros::Time::now();
          ros::spinOnce();
          r.sleep();
      }
        ROS_INFO("A for pulling");
  
  //get the 3dhandle
  ros::ServiceClient client_1 = n.serviceClient<tf_tiago::doorshaft>("handle3d");
    tf_tiago::doorshaft srv_1;
    srv_1.request.req = 1;
    if (client_1.call(srv_1))
      {
          doorshaft_x = srv_1.response.doorshaft_res.x;
          doorshaft_y = srv_1.response.doorshaft_res.y;
          doorshaft_z = srv_1.response.doorshaft_res.z;
          theta = srv_1.response.theta_res;
          handle_x = srv_1.response.handle_res.x;
          handle_y = srv_1.response.handle_res.y;
          handle_z = srv_1.response.handle_res.z;

          ROS_INFO_STREAM("I'got"<<srv_1.response);
      }
    else
      {
        ROS_ERROR("Failed to 3Dposition");
        return 1;
      }
        
  /******************************************************************************************************************************************/

  /*Moving the robot arm to a pre-position and grasping handle

  /******************************************************************************************************************************************/   
  ros::ServiceClient client_2 = n.serviceClient<ikTiago::gripper>("little_move");
  ikTiago::gripper srv_2;
  srv_2.request.gripper_req.x = handle_x;
  srv_2.request.gripper_req.y = handle_y-0.10;//handle_y
  srv_2.request.gripper_req.z = -0.14427;
  srv_2.request.theta = theta;
  if (client_2.call(srv_2))
    {
      ROS_INFO_STREAM("I'm moving to pre position");
    }
  else
    {
        ROS_ERROR("Failed to find a plan");
        return 1;
    }


  //tune the gripper orientation
  ros::ServiceClient client_3 = n.serviceClient<ikTiago::gripper>("little_move");
  ikTiago::gripper srv_3;
  srv_3.request.gripper_req.x = handle_x;
  srv_3.request.gripper_req.y = handle_y-0.06;
  srv_3.request.gripper_req.z = -0.14427;
  srv_3.request.theta = theta;
  if (client_3.call(srv_3))
    {
      ROS_INFO_STREAM("I've already grasped the handle and");
    }
  else
    {
        ROS_ERROR("Failed to find a plan");
        return 1;
    }
      
  /******************************************************************************************************************************************/

  /*Call the trajectory("/opendoor") service to open the door and Take the hand out of the handle

  /******************************************************************************************************************************************/ 
  
  ros::ServiceClient client_4 = n.serviceClient<ikTiago::doorinfo>("/opendoor");
  ikTiago::doorinfo srv_4;
  srv_4.request.door_req.x = doorshaft_x;
  srv_4.request.door_req.y = doorshaft_y;//handle_y
  srv_4.request.door_req.z = doorshaft_z;
  srv_4.request.theta = theta;
  if (client_4.call(srv_4))
    {
      ROS_INFO_STREAM("Start to open the door");
    }
  else
    {
        ROS_ERROR("Failed to find a plan");
        return 1;
    }
        
  /******************************************************************************************************************************************/

  /*Open teh door widely

  /******************************************************************************************************************************************/
  ros::Time ti_2, tf_2;
  ti_2=ros::Time::now();
  tf_2=ti_2;
  while((tf_2.toSec()-ti_2.toSec())<=5)
  {    
      tf_2=ros::Time::now();
      ros::spinOnce();
      r.sleep();
  }
  ROS_INFO("finish taking hand out of the door ");

  //back to home position
  system("rosrun tiago_gazebo tuck_arm.py");
  //back to pre-position and close the gripper
  std_srvs::Empty srv_empty_gripper; 
  ros::service::call("/gripper_controller/grasp", srv_empty_gripper);
  lifter.action(pose);
      
  ros::ServiceClient client_5 = n.serviceClient<ikTiago::gripper>("openwide");
  ikTiago::gripper srv_5;
  srv_5.request.gripper_req.x = handle_x+0.03;
  srv_5.request.gripper_req.y = handle_y-0.07;//handle_y
  srv_5.request.gripper_req.z = -0.14427;
  srv_5.request.theta = theta;
  if (client_5.call(srv_5))
    {
      ROS_INFO_STREAM("Start to open the door widely");
    }
  else
    {
        ROS_ERROR("Failed to find a plan");
        return 1;
    }
  
  /******************************************************************************************************************************************/

  /*Back to home for object grasping and waiting for 3D position of the bottle computed by YOLO

  /******************************************************************************************************************************************/
  system("rosrun tiago_gazebo tuck_arm.py");
  system("rosrun pal_gripper_controller_configuration_gazebo home_gripper.py");
  
  ros::Time ti_tem_0, tf_tem_0;
    ti_tem_0=ros::Time::now();
      tf_tem_0=ti_tem_0;
      while((tf_tem_0.toSec()-ti_tem_0.toSec())<=5)
    {   
      tf_tem_0=ros::Time::now();
      ROS_INFO("Reaching A ");
      ros::spinOnce();
      r.sleep();
    }
  //start from2Dto3D to get position     
  ros::ServiceClient client_6 = n.serviceClient<from2Dto3D::ReachGoal>("ReachCheck");
  from2Dto3D::ReachGoal srv_6;
  srv_6.request.req = 1;
  if (client_6.call(srv_6))
    {         
        ROS_INFO_STREAM("succeed to launch from2Dto3D");
    }
  else
    {
      ROS_ERROR("Failed to launch frome2Dto3D");
      return 1;
    }

  ros::Time ti_tem_1, tf_tem_1;
    ti_tem_1=ros::Time::now();
      tf_tem_1=ti_tem_1;
      while((tf_tem_1.toSec()-ti_tem_1.toSec())<=35)
    {   
      tf_tem_1=ros::Time::now();
      ROS_INFO("I'm waiting 3D ");
      ros::spinOnce();
      r.sleep();
    }
  ROS_INFO("A for grasping");

  ros::ServiceClient client = n.serviceClient<from2Dto3D::PointBase>("PoseBaselink");
  from2Dto3D::PointBase srv;
  srv.request.req = 1;
  if (client.call(srv))
    {
        tiago::x_3d=srv.response.pose.x;
        tiago::y_3d=srv.response.pose.y;
        tiago::z_3d=srv.response.pose.z;         
        ROS_INFO_STREAM("I'got"<<srv.response);
    }
  else
    {
      ROS_ERROR("Failed to 3Dposition");
      return 1;
    }
  /******************************************************************************************************************************************/

  /*Move to pre-position for object grasping

  /******************************************************************************************************************************************/
  double pose1[8]={0.15, 0.2, -1.34, -0.2, 1.94, -1.57, 1.37, 0.0};
  lifter.action(pose1);
  pose1[2]= -0.21;
  pose1[3]= -1.54;
  pose1[4]= 2.08;
  lifter.action(pose1);
  pose1[5]= -1.73;
  pose1[6]= -0.94;
  pose1[7]= -1.68;
  pose1[1]= 0.44;
  lifter.action(pose1);
  /******************************************************************************************************************************************/

  /*Tuning to grasp the bottle and go back to home position

  /******************************************************************************************************************************************/  
  double px,py,pz;
  double xbias=0.13;
  double ybias=0.02;
  px=tiago::x_3d-xbias;
  py=tiago::y_3d-ybias;
  pz=tiago::z_3d;
  double handpose[3]={0.557298,py,pz};
  lifter.movejoint(handpose);
  handpose[0]=px;
  lifter.movejoint(handpose);
  //grasp
  std_srvs::Empty srv_empty_gripper1; 
  ros::service::call("/gripper_controller/grasp", srv_empty_gripper1);

//tuning to a safe pose to home 
  double handpose1[3]={0.557298,py,pz};
  lifter.movejoint(handpose1);

  system("rosrun tiago_gazebo tuck_arm.py");

  ros::spin();
  return 0;
}
