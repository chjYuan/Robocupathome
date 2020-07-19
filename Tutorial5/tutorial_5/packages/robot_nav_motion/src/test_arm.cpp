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

float x_3d;
float y_3d;
float z_3d;
void Callback_3D(const geometry_msgs::Point::ConstPtr& msg)
    {
        /******get the current position of object****/
        x_3d=msg->x;
        y_3d=msg->y;
        z_3d=msg->z;
        ROS_INFO_STREAM("I heard: [%s]"<<x_3d );
        ROS_INFO_STREAM("I heard: [%s]"<<y_3d );
        ROS_INFO_STREAM("I heard: [%s]"<<z_3d );
    }
int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_arm");
  ros::NodeHandle n;
  ros::Rate r(60);

  /******************************************************************************************************************************************/

  /*Get the current 3-D Position from Camera

  /******************************************************************************************************************************************/ 
  ros::ServiceClient client = n.serviceClient<from2Dto3D::PointBase>("PoseBaselink");
    from2Dto3D::PointBase srv;
    srv.request.req = 1;
    if (client.call(srv))
      {
         x_3d=srv.response.pose.x;
         y_3d=srv.response.pose.y;
         z_3d=srv.response.pose.z;
         
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
  //double pose[8]={0.15, 0.2, -1.34, -0.2, 1.94, -1.57, 0.0, 0.0};//home
  //double pose[8]={0.15, 0.2, -1.34, -0.2, 1.94, -1.57, 0.0, 0.0};
  //lifter.action(pose);
    /* ROS_INFO_STREAM("111111111111");
   
    ros::Subscriber sub = n.subscribe("base_point3D", 10, Callback_3D);
    ROS_INFO_STREAM("22222222222222");*/    
     
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
     px=x_3d-xbias;
     py=y_3d-ybias;
     pz=z_3d;
     double handpose[3]={0.557298,py,pz};
     lifter.movejoint(handpose);
     handpose[0]=px;
     lifter.movejoint(handpose);
     
     
     std_srvs::Empty srv_empty_gripper; 
     ros::service::call("/gripper_controller/grasp", srv_empty_gripper);

     
    /******************************************************************************************************************************************/

    /*Moving back a certain distance B for Home pose (90 degree rotation)

    /******************************************************************************************************************************************/
    tiago::pub=n.advertise< geometry_msgs::Twist>("/mobile_base_controller/cmd_vel",100);
      ros::Time ti, tf;
      ti=ros::Time::now();
       tf=ti;
       geometry_msgs::Twist msg;
       while((tf.toSec()-ti.toSec())<=5)
     {   
    
       tf=ros::Time::now();
       msg.angular.z=0.5;//make robot self rotate
       tiago::pub.publish(msg);
       ros::spinOnce();
     r.sleep();
     }
    
    //tigao::navigation(2.79406978607,-0.1003755102158,0.0,0.0,0.009548555123, 0.999954411508);
    /*double pose_2[8]={0.19, 0.44, -0.21, -1.54, 2.08, -1.73, -0.94, -1.68};    
     lifter.action(pose_2);
     pose_2[0]=0.24;
     lifter.action(pose_2);
     pose_2[1]=1.08;
     pose_2[4]= 1.10;
     pose_2[6]= -0.44;
     pose_2[7]= -1.58;
     lifter.action(pose_2);  */
    ROS_INFO("B for homing");
    system("rosrun tiago_gazebo tuck_arm.py ");
   
    
    /******************************************************************************************************************************************/

    /*Moving the robot arm to a pre-position for placing

    /******************************************************************************************************************************************/ 
    /*   jointspace::jointlift lifter;
     double pose_1[8]={0.19, 0.2, -1.34, -0.2, 1.94, -1.57, 0.0, 0.0};
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
     */
     /*system("rosrun pal_gripper_controller_configuration_gazebo home_gripper.py");*/


     
     
     ros::spin();
     return 0;
}
