#include <boost/date_time.hpp>
#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>
#include <tf_tiago/doorshaft.h>
#include <tf_tiago/ReachGoal.h>
#include <ikTiago/gripper.h>
#include <ikTiago/doorinfo.h>


//**************
// C++ standard headers
#include <exception>
#include <string>

// Boost headers
#include <boost/shared_ptr.hpp>

// ROS headers
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/topic.h>

const double PI=3.1415926;
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_control_client;
typedef boost::shared_ptr< arm_control_client>  arm_control_client_Ptr;
void test(ros::NodeHandle& nh, std::string urdf_param, double theta, double x, double y, double z);
bool reachCallback(ikTiago::doorinfo::Request& req, ikTiago::doorinfo::Response& res);
int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "ikTiago");
  
  
  // double theta,x,y,z;
  // theta=0.386875;
  // x=1.01994;
  // y=0.33171;
  // z=-0.1427; 
  // ros::NodeHandle nh;
  // std::string urdf_param;
  // nh.param("urdf_param", urdf_param, std::string("/robot_description"));
  // test(nh, urdf_param, theta,x, y, z);
  
  ros::NodeHandle nh;
  ros::ServiceServer get_transform;
  get_transform=nh.advertiseService("/opendoor", reachCallback);


  ros::spin();
  

  
  return 0;
}
bool reachCallback(ikTiago::doorinfo::Request& req, ikTiago::doorinfo::Response& res)
{ 
    ros::NodeHandle n;
    std::string urdf_param;
    n.param("urdf_param", urdf_param, std::string("/robot_description"));
    test(n, urdf_param, req.theta, req.door_req.x, req.door_req.y, req.door_req.z);
    res.response=true;
    return res.response;
}

void createArmClient(arm_control_client_Ptr& actionClient)
{
  ROS_INFO("Creating action client to arm controller ...");

  actionClient.reset( new arm_control_client("/arm_controller/follow_joint_trajectory") );

  int iterations = 0, max_iterations = 3;
  // Wait for arm controller action server to come up
  while( !actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
  {
    ROS_DEBUG("Waiting for the arm_controller_action server to come up");
    ++iterations;
  }

  if ( iterations == max_iterations )
    throw std::runtime_error("Error in createArmClient: arm controller action server not available");
}

void test(ros::NodeHandle& nh, std::string urdf_param, double theta, double x, double y, double z)
{
  ROS_INFO("Starting run_traj_control application ...");
  if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    
  }

  // Create an arm controller action client to move the TIAGo's arm
  arm_control_client_Ptr ArmClient;
  createArmClient(ArmClient);
  
  
  // Generates the goal for the TIAGo's arm
  
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory.joint_names.push_back("arm_1_joint");
  goal.trajectory.joint_names.push_back("arm_2_joint");
  goal.trajectory.joint_names.push_back("arm_3_joint");
  goal.trajectory.joint_names.push_back("arm_4_joint");
  goal.trajectory.joint_names.push_back("arm_5_joint");
  goal.trajectory.joint_names.push_back("arm_6_joint");
  goal.trajectory.joint_names.push_back("arm_7_joint");

  int waypointsnumber=40;

  goal.trajectory.points.resize(waypointsnumber);
  
  /************************************************
                TRAC_IT initialization
  ************************************************/
  double eps = 1e-5;
  double timeout = 0.05;
  std::string chain_start, chain_end;
  chain_start = "torso_lift_link";
  chain_end = "arm_tool_link";
  TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, eps);
  KDL::Chain chain;
  bool valid = tracik_solver.getKDLChain(chain);  //load chain
  if (!valid)
  {
    ROS_ERROR("There was no valid KDL chain found");
  }  
  else 
    ROS_INFO_STREAM("KDL chain is loaded successfully");

  /************************************************
                TRAC_IT initialization
  ************************************************/
    
  /**********************************************************************
       Trajctory in cartesian space generation & inverse kinematics
  **********************************************************************/
  
  KDL::JntArray Jointstate0(chain.getNrOfJoints());
  Jointstate0(0)=0.53;
  Jointstate0(1)=0.11;
  Jointstate0(2)=-1.58;
  Jointstate0(3)=1.07;
  Jointstate0(4)=-1.46;
  Jointstate0(5)=1,23;
  Jointstate0(6)=1.57;
  

  //frame arm_tool_link wrt door_frame
  KDL::Rotation my_rotation1 = KDL::Rotation::RPY(PI/2,0,PI);
  KDL::Vector my_vector1(0.56,-0.08,0); //here should add a bias corresponding to size of the grisper
  KDL::Frame doorshaft_wrt_gripper(my_rotation1,my_vector1); 
  
  // F_A_C.p is the location of the origin of frame C expressed in frame A, and F_A_C.M is the rotation of frame C expressed in frame A. 
  
  for(int i=0;i<waypointsnumber;i++)
  {
    
    KDL::Rotation my_rotation2 = KDL::Rotation::RPY(0,0,0-PI/2-theta);
    KDL::Vector my_vector2(x,y,-0.1427);
    KDL::Frame torsoliftlink_wrt_doorshaft(my_rotation2,my_vector2); 
    KDL::Frame torsoliftlink_wrt_gripper = torsoliftlink_wrt_doorshaft*doorshaft_wrt_gripper;
    //std::cout<<torsoliftlink_wrt_gripper<<std::endl;

    //tf doorshaft
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(x, y, -0.1427) );
    tf::Quaternion q;
    q.setRPY(0,0,0-PI/2-theta);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/torso_lift_link", "doorshaft"));
    
    //  tf gripper1
    static tf::TransformBroadcaster br1;
    transform.setOrigin( tf::Vector3(0.56,-0.08,0) );
    q.setRPY(0,0,PI);
    transform.setRotation(q);
    br1.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "doorshaft", "gripper1"));

    // tf gtipper2
    ROS_INFO_STREAM(i+1<<"times");
    ROS_INFO_STREAM(torsoliftlink_wrt_gripper(0,0)<<" "<<torsoliftlink_wrt_gripper(0,1)<<" "<<torsoliftlink_wrt_gripper(0,2)<<" "<<torsoliftlink_wrt_gripper(0,3));
    ROS_INFO_STREAM(torsoliftlink_wrt_gripper(1,0)<<" "<<torsoliftlink_wrt_gripper(1,1)<<" "<<torsoliftlink_wrt_gripper(1,2)<<" "<<torsoliftlink_wrt_gripper(1,3));
    ROS_INFO_STREAM(torsoliftlink_wrt_gripper(2,0)<<" "<<torsoliftlink_wrt_gripper(2,1)<<" "<<torsoliftlink_wrt_gripper(2,2)<<" "<<torsoliftlink_wrt_gripper(2,3));
    

    //sleep(1);
    
    
    KDL::JntArray result; //the result of the inverse kinematics
    int rc; //retrun value, that refers to the result of inverse kinematics
    rc = tracik_solver.CartToJnt(Jointstate0, torsoliftlink_wrt_gripper, result);
    
    int index=i;
    if (rc>=0)
    {
      theta=theta+0.01;
      for (int k=0;k<7;k++)
      {
        Jointstate0(k)=result(k);
      }
      ROS_INFO_STREAM(result(0)<<" "<<result(1)<<" "<<result(2)<<" "<<result(3)<<" "<<result(4)<<" "<<result(5)<<" "<<result(6));

      //// Generates the goal for the TIAGo's arm
      
      // set positions
      
      goal.trajectory.points[index].positions.resize(7);
      
      goal.trajectory.points[index].positions[0] = result(0);
      
      goal.trajectory.points[index].positions[1] = result(1);
      goal.trajectory.points[index].positions[2] = result(2);
      goal.trajectory.points[index].positions[3] = result(3);
      goal.trajectory.points[index].positions[4] = result(4);
      goal.trajectory.points[index].positions[5] = result(5);
      goal.trajectory.points[index].positions[6] = result(6);
      //ROS_INFO_STREAM("KKKKKKKKKKKKK");
      // set velocity
      goal.trajectory.points[index].velocities.resize(7);
      if (index<waypointsnumber-1)
      {
        for (int j = 0; j < 7; ++j)
        {
          goal.trajectory.points[index].velocities[j] = 0.0;
        }
        
      }
        
      else
      {
        for (int j = 0; j < 7; ++j)
        {
          goal.trajectory.points[index].velocities[j] = 0.0;
        }
      }
      goal.trajectory.points[index].time_from_start = ros::Duration(index*0.5+0.5);

    }
    else 
      {
        ROS_ERROR("The %d time ik is failed.", (i+1));
        //theta-=0.01;
        goal.trajectory.points[index].positions.resize(7);
        ROS_INFO_STREAM("DDDDDDDDDDDDDDDD");
        goal.trajectory.points[index].positions[0] =  Jointstate0(0);
        ROS_INFO_STREAM("111111111111111");
        goal.trajectory.points[index].positions[1] =  Jointstate0(1);
        goal.trajectory.points[index].positions[2] =  Jointstate0(2);
        goal.trajectory.points[index].positions[3] =  Jointstate0(3);
        goal.trajectory.points[index].positions[4] =  Jointstate0(4);
        goal.trajectory.points[index].positions[5] =  Jointstate0(5);
        goal.trajectory.points[index].positions[6] =  Jointstate0(6);
        //ROS_INFO_STREAM("22222222222222222222");
        goal.trajectory.points[index].velocities.resize(7);
        for (int j = 0; j < 7; ++j)
        {
          goal.trajectory.points[index].velocities[j] = 0.0;
        }
        //ROS_INFO_STREAM("333333333333333333333");
        goal.trajectory.points[index].time_from_start = ros::Duration(index*0.5+0.5);

        //ROS_INFO_STREAM("DDDDDDDDDDDDDDDD");
      }


    
  }

  /*****************************************************************************************
                                  Joint trajctory control                 
  *****************************************************************************************/
  

  // Sends the command to start the given trajectory 1s from now
  goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
  ArmClient->sendGoal(goal);

  // Wait for trajectory execution
  while(!(ArmClient->getState().isDone()) && ros::ok())
  {
    ros::Duration(4).sleep(); // sleep for four seconds
  }
  

  
  /*****************************************************************************************
                                  Joint trajctory control                 
  *****************************************************************************************/

  //*******************************************************************************************
  // take the hand back
  //*******************************************************************************************
  
  ROS_INFO_STREAM("*********************");
  int times=15;
  control_msgs::FollowJointTrajectoryGoal waypoint;
  waypoint.trajectory.joint_names.push_back("arm_1_joint");
  waypoint.trajectory.joint_names.push_back("arm_2_joint");
  waypoint.trajectory.joint_names.push_back("arm_3_joint");
  waypoint.trajectory.joint_names.push_back("arm_4_joint");
  waypoint.trajectory.joint_names.push_back("arm_5_joint");
  waypoint.trajectory.joint_names.push_back("arm_6_joint");
  waypoint.trajectory.joint_names.push_back("arm_7_joint");
  
  waypoint.trajectory.points.resize(times);
  for(int l=0;l<times;l++)
  {
    //  position
    waypoint.trajectory.points[l].positions.resize(7);
    waypoint.trajectory.points[l].positions[0] = Jointstate0(0)-l*0.013;
    waypoint.trajectory.points[l].positions[1] = Jointstate0(1);
    waypoint.trajectory.points[l].positions[2] = Jointstate0(2);
    waypoint.trajectory.points[l].positions[3] = Jointstate0(3);
    waypoint.trajectory.points[l].positions[4] = Jointstate0(4);
    waypoint.trajectory.points[l].positions[5] = Jointstate0(5)-l*0.0075;
    waypoint.trajectory.points[l].positions[6] = Jointstate0(6);
    
    
    //  velocity
    waypoint.trajectory.points[l].velocities.resize(7);
    for (int kk=0;kk<7;kk++)
    {
      waypoint.trajectory.points[l].velocities[kk] = 0.0;
    }
    
    //  duration
    waypoint.trajectory.points[l].time_from_start = ros::Duration(l*0.5+0.5);

  }

  /*****************************************************************************************
                                  Joint trajctory control                 
  *****************************************************************************************/
  

  // Sends the command to start the given trajectory 1s from now
  waypoint.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
  ArmClient->sendGoal(waypoint);

  // Wait for trajectory execution
  while(!(ArmClient->getState().isDone()) && ros::ok())
  {
    ros::Duration(4).sleep(); // sleep for four seconds
  }
  

  
  /*****************************************************************************************
                                  Joint trajctory control                 
  *****************************************************************************************/


  

  //*******************************************************************************************
  // take the hand back
  //*******************************************************************************************
  
  //  for(int m=0; m<6 ;m++)
  //  {
  //   KDL::Rotation my_rotation3 = KDL::Rotation::RPY(PI/2,0.05*m,PI);
  //   KDL::Vector my_vector3(0.54+m*0.02,-0.08,0); //here should add a bias corresponding to size of the grisper
  //   KDL::Frame doorshaft_wrt_gripper3(my_rotation3,my_vector3);
    
  //   KDL::Rotation my_rotation4 = KDL::Rotation::RPY(0,0,0-PI/2-theta);
  //   KDL::Vector my_vector4(x,y,-0.1427);
  //   KDL::Frame torsoliftlink_wrt_doorshaft4(my_rotation4,my_vector4);
  //   KDL::Frame torsoliftlink_wrt_gripper4 = torsoliftlink_wrt_doorshaft4*doorshaft_wrt_gripper3;

  //   KDL::JntArray result1; //the result of the inverse kinematics
  //   int rc1; //retrun value, that refers to the result of inverse kinematics
  //   rc1 = tracik_solver.CartToJnt(Jointstate0, torsoliftlink_wrt_gripper4, result1);
  
  //   // int index=m;
  //   if (rc1>=0)
  //   {
      
  //     for (int k=0;k<7;k++)
  //       {
  //         Jointstate0(k)=result1(k);
  //       }
  //     // ROS_INFO_STREAM(result(0)<<" "<<result(1)<<" "<<result(2)<<" "<<result(3)<<" "<<result(4)<<" "<<result(5)<<" "<<result(6));

  //     //// Generates the goal for the TIAGo's arm
      
  //     // set positions
      
  //     goal.trajectory.points[index+(m+1)].positions.resize(7);
      
  //     goal.trajectory.points[index+(m+1)].positions[0] = result1(0);
      
  //     goal.trajectory.points[index+(m+1)].positions[1] = result1(1);
  //     goal.trajectory.points[index+(m+1)].positions[2] = result1(2);
  //     goal.trajectory.points[index+(m+1)].positions[3] = result1(3);
  //     goal.trajectory.points[index+(m+1)].positions[4] = result1(4);
  //     goal.trajectory.points[index+(m+1)].positions[5] = result1(5);
  //     goal.trajectory.points[index+(m+1)].positions[6] = result1(6);
  //     //ROS_INFO_STREAM("KKKKKKKKKKKKK");
  //     // set velocity
  //     goal.trajectory.points[index+(m+1)].velocities.resize(7);
      
  //     for (int j = 0; j < 7; ++j)
  //     {
  //       goal.trajectory.points[index+(m+1)].velocities[j] = 0.0;
  //     }
  //     goal.trajectory.points[index+(m+1)].time_from_start = ros::Duration(index*1+1);

  //   }
  //   else 
  //   {
  //     ROS_ERROR("The %d time ik is failed.", (+1));
  //     //theta-=0.01;
  //     goal.trajectory.points[index+(m+1)].positions.resize(7);
  //     ROS_INFO_STREAM("DDDDDDDDDDDDDDDD");
  //     goal.trajectory.points[index+(m+1)].positions[0] =  Jointstate0(0);
  //     ROS_INFO_STREAM("111111111111111");
  //     goal.trajectory.points[index+(m+1)].positions[1] =  Jointstate0(1);
  //     goal.trajectory.points[index+(m+1)].positions[2] =  Jointstate0(2);
  //     goal.trajectory.points[index+(m+1)].positions[3] =  Jointstate0(3);
  //     goal.trajectory.points[index+(m+1)].positions[4] =  Jointstate0(4);
  //     goal.trajectory.points[index+(m+1)].positions[5] =  Jointstate0(5);
  //     goal.trajectory.points[index+(m+1)].positions[6] =  Jointstate0(6);
  //     ROS_INFO_STREAM("22222222222222222222");
  //     goal.trajectory.points[index+(m+1)].velocities.resize(7);
  //     for (int j = 0; j < 7; ++j)
  //     {
  //       goal.trajectory.points[index+(m+1)].velocities[j] = 0.0;
  //     }
  //     ROS_INFO_STREAM("333333333333333333333");
  //     goal.trajectory.points[index+(m+1)].time_from_start = ros::Duration(index*1+1);

  //     ROS_INFO_STREAM("DDDDDDDDDDDDDDDD");
  //   }

  // }
  

  
  
}
