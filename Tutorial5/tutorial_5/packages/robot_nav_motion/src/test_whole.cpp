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

/*typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;*/

int main(int argc, char** argv){
  ros::init(argc, argv, "test_whole");
  ros::NodeHandle n;
  ros::Rate r(60);
 
  /******************************************************************************************************************************************/

  /*Rotating for Localization

  /******************************************************************************************************************************************/
  
  //ros::Publisher pub=n.advertise< geometry_msgs::Twist>("/mobile_base_controller/cmd_vel",100);
  tiago::pub=n.advertise< geometry_msgs::Twist>("/mobile_base_controller/cmd_vel",100);
  //ros::Publisher pub_2=n.advertise< geometry_msgs::Twist>("/mobile_base_controller/cmd_vel",100);
  //ros::Publisher pub_3=n.advertise< geometry_msgs::Twist>("/mobile_base_controller/cmd_vel",100);

  double t_rotate;
  std_srvs::Empty srv_empty; 
  //wait for the services to come up
  ros::Duration(5.0).sleep();
  // locate the robot in the world
  ros::service::call("/global_localization", srv_empty);
  
  ros::Time ti_0, tf_0;
  ti_0=ros::Time::now();
  tf_0=ti_0;
  geometry_msgs::Twist msg_0;
  while((tf_0.toSec()-ti_0.toSec())<=30)
  {   
    
     tf_0=ros::Time::now();
       msg_0.angular.z=3.5;//make robot self rotate
       tiago::pub.publish(msg_0);
     ros::spinOnce();
     r.sleep();
  }
  
  ros::service::call("/move_base/clear_costmaps", srv_empty);
  ROS_INFO("Localization finished");
  
   
  /******************************************************************************************************************************************/

  /*Moving the robot to position A for grasping

  /******************************************************************************************************************************************/
  
    ROS_INFO("A ready for grasping");
                      //-1.07883024216,-0.214169025421         -0.857760062798;    0.51405026473;
                      //-3.50932645798;-0.815170049667;         0.308702930911;    0.951158504376;
    //tiago::navigation(-1.07883024216,-0.214169025421,0.0,0.0,-0.857760062798,0.51405026473); gazeobotest
    tiago::navigation(2.77406978607,-0.993755102158,0.0,0.0,-0.791229184193,0.611519728285);//-0.747944095454 ,0.663761726884
    
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

    //launch the from2Dto3D
    ros::ServiceClient client_0 = n.serviceClient<from2Dto3D::ReachGoal>("ReachCheck");
    from2Dto3D::ReachGoal srv_0;
    srv_0.request.req = 1;
    if (client_0.call(srv_0))
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
       while((tf_tem_1.toSec()-ti_tem_1.toSec())<=60)
     {   
       tf_tem_1=ros::Time::now();
       ROS_INFO("I'm waiting 3D ");
       ros::spinOnce();
       r.sleep();
     }
    ROS_INFO("A for grasping");

  /******************************************************************************************************************************************/

  /*Get the current 3-D Position from Camera

  /******************************************************************************************************************************************/  
    
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

  /*Moving the robot arm to a pre-position for grasping

  /******************************************************************************************************************************************/
    
     tiago::jointlift lifter;
     double pose[8]={0.15, 0.2, -1.34, -0.2, 1.94, -1.57, 0.0, 0.0};
     lifter.action(pose);
     pose[2]= -0.21;
     pose[3]= -1.54;
     pose[4]= 2.08;
     lifter.action(pose);
     pose[5]= -1.73;
     pose[6]= -0.94;
     pose[7]= -1.68;
     pose[1]= 0.44;
     lifter.action(pose);
  
  /******************************************************************************************************************************************/

  /*Grasping the object by the current 3-D Position

  /******************************************************************************************************************************************/   

     double px,py,pz;
     double xbias=0.1;
     double ybias=0.02;
     px=tiago::x_3d-xbias;
     py=tiago::y_3d-ybias;
     pz=tiago::z_3d;
     double handpose[3]={0.557298,py,pz};
     lifter.movejoint(handpose);
     handpose[0]=px;
     lifter.movejoint(handpose);
     //grasp
     std_srvs::Empty srv_empty_gripper; 
     ros::service::call("/gripper_controller/grasp", srv_empty_gripper);
 
  /******************************************************************************************************************************************/

  /*Moving back a certain distance B for Home pose (90 degree rotation)

  /******************************************************************************************************************************************/
  
      ros::Time ti_1, tf_1;
      ti_1=ros::Time::now();
      tf_1=ti_1;
      geometry_msgs::Twist msg_1;
       while((tf_1.toSec()-ti_1.toSec())<=4)
     {   
    
       tf_1=ros::Time::now();
       msg_1.angular.z=0.5;//make robot self rotate
       tiago::pub.publish(msg_1);
       ros::spinOnce();
       r.sleep();
     }
      //rotate(4,0.5);
      ROS_INFO("B for homing");
      system("rosrun tiago_gazebo tuck_arm.py "); 

  /******************************************************************************************************************************************/

  /*Moving the robot  to a place-position C for placing

  /******************************************************************************************************************************************/
 
    ROS_INFO("C ready for placing");

    tiago::navigation(4.00103660583,-1.18337025642,0.0,0.0,-0.791229184193,0.611519728285);
    
    ROS_INFO("C for placing");
  
  /******************************************************************************************************************************************/

  /*Moving the robot arm to a pre-position for placing

  /******************************************************************************************************************************************/
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
     pose_2[0]=0.28;
     lifter.action(pose_2);
     pose_2[1]=1.08;
     pose_2[4]= 1.10;
     pose_2[6]= -0.44;
     pose_2[7]= -1.58;
     lifter.action(pose_2);  
     //release the gripper
    
     system("rosrun pal_gripper_controller_configuration_gazebo home_gripper.py");

     //lift the torso for rotate
     
     double pose_3[8]={0.28, 1.08, -0.21, -1.54, 1.10, -1.73, -0.44, -1.58};    
     pose_3[0]=0.35;
     lifter.action(pose_3);
     pose_1[0]= 0.35;
     lifter.action(pose_1);
     
      ros::Time ti_2, tf_2;
      ti_2=ros::Time::now();
       tf_2=ti_2;
       geometry_msgs::Twist msg_2;
       while((tf_2.toSec()-ti_2.toSec())<=4)
     {   
    
       tf_2=ros::Time::now();
       msg_2.angular.z=0.5;//make robot self rotate
       tiago::pub.publish(msg_2);
       ros::spinOnce();
     r.sleep();
     }
      //rotate(4,0.5);
      ROS_INFO("D for homing");
      system("rosrun tiago_gazebo tuck_arm.py ");  
  
  ros::spin();
  return 0;
}