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


using namespace tiago;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_nav");
  ros::NodeHandle n;
  ros::Rate r(60);
  geometry_msgs::Twist msg;
  //tell the action client that we want to spin a thread by default
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
  
  ros::service::call("/global_localization", srv_empty);
  
  while((tf.toSec()-ti.toSec())<=20)
  {   
    
     tf=ros::Time::now();
       msg.angular.z=2.5;//make robot self rotate
       pub.publish(msg);
     ros::spinOnce();
     r.sleep();
  }
  ros::service::call("/move_base/clear_costmaps", srv_empty);
  ROS_INFO("Localization finished");
  /******************************************************************************************************************************************/

  /*Moving the robot arm to a pre-position A for grasping

  /******************************************************************************************************************************************/

  tiago::navigation(-1.17319154739 ,2.29208159447,0.0,0.0,-0.999949607371,0.0100390596851);

  tiago::navigation(-2.27096295357,2.36211061478,0.0,0.0,-0.999828141019,0.0185388356184);

  ROS_INFO("A for pulling");
 
  


      return 0;
}
