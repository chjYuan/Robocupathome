#include <boost/date_time.hpp>
#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>
#include <tf_tiago/doorshaft.h>
#include <tf_tiago/ReachGoal.h>
#include <tf_tiago/doorshaft.h>
#include <ikTiago/gripper.h>

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
bool reachCallback(ikTiago::gripper::Request& req, ikTiago::gripper::Response& res);
int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "little_move");
  ros::NodeHandle nh; 
  ros::ServiceServer get_transform;
  get_transform=nh.advertiseService("/little_move", reachCallback);
  ros::spin();
  return 0;
}

bool reachCallback(ikTiago::gripper::Request& req, ikTiago::gripper::Response& res)
{ 
    ros::NodeHandle n;
    std::string urdf_param;
    n.param("urdf_param", urdf_param, std::string("/robot_description"));
    test(n, urdf_param, req.theta, req.gripper_req.x, req.gripper_req.y, req.gripper_req.z);
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


// this function is used to calculate the trajectory of the handle and execute the inverse kinematics,
// for this part, the Trak_ik is used to instead the original 
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
  int waypointsnumber=1;
  goal.trajectory.points.resize(1);
  
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

  /*****************double*******************************
                TRAC_IT initialization
  ************************************************/
    
  /**********************************************************************
       Trajctory in cartesian space generation & inverse kinematics
  **********************************************************************/
  
  KDL::JntArray Jointstate0(chain.getNrOfJoints());
  Jointstate0(0)=0.26;
  Jointstate0(1)=0.01;
  Jointstate0(2)=-1.58;
  Jointstate0(3)=1.09;
  Jointstate0(4)=-1.44;
  Jointstate0(5)=1.39;
  Jointstate0(6)=1.57;
    
      
  KDL::Rotation my_rotation2 = KDL::Rotation::RPY(PI/2,0,PI/2-theta);
  KDL::Vector my_vector2(x,y,z);
  KDL::Frame torsoliftlink_wrt_gripper(my_rotation2,my_vector2); 
  // KDL::Frame torsoliftlink_wrt_gripper = torsoliftlink_wrt_doorshaft*doorshaft_wrt_gripper;


  //theta=theta+0.01;
    
    KDL::JntArray result; //the result of the inverse kinematics
    int rc; //retrun value, that refers to the result of inverse kinematics
    rc = tracik_solver.CartToJnt(Jointstate0, torsoliftlink_wrt_gripper, result);
    if (rc>=0)
    {
      for (int k=0;k<7;k++)
      {
        Jointstate0(k)=result(k);
      }
      ROS_INFO_STREAM(result(0)<<" "<<result(1)<<" "<<result(2)<<" "<<result(3)<<" "<<result(4)<<" "<<result(5)<<" "<<result(6));

      //// Generates the goal for the TIAGo's arm
      int index=0;
      // set positions
      //ROS_INFO_STREAM("FFFFFFFFFFFFFFFF");
      goal.trajectory.points[index].positions.resize(7);
      //ROS_INFO_STREAM("KKKKKKKKKKKKK");
      goal.trajectory.points[index].positions[0] = result(0);
      //ROS_INFO_STREAM("KKKKKKKKKKKKK");
      goal.trajectory.points[index].positions[1] = result(1);
      goal.trajectory.points[index].positions[2] = result(2);
      goal.trajectory.points[index].positions[3] = result(3);
      goal.trajectory.points[index].positions[4] = result(4);
      goal.trajectory.points[index].positions[5] = result(5);
      goal.trajectory.points[index].positions[6] = result(6);
      //ROS_INFO_STREAM("KKKKKKKKKKKKK");
      // set velocity
      goal.trajectory.points[index].velocities.resize(7);
      // if (index<waypointsnumber-1)
      // {
         for (int j = 0; j < 7; ++j)
         {
          goal.trajectory.points[index].velocities[j] = 0.01;
         }
        
      // }        
      // else
      // {
      //   for (int j = 0; j < 7; ++j)
      //   {
      //     goal.trajectory.points[index].velocities[j] = 0.0;
      //   }
      // }
      goal.trajectory.points[index].time_from_start = ros::Duration(index*1+5);

    }
    else 
      ROS_ERROR("The %d time ik is failed.", (0+1));
    //ROS_INFO_STREAM("DDDDDDDDDDDDDDDD");
  //}


  /*****************************************************************************************
                                  Joint trajctory control                 
  *****************************************************************************************/
  

  // // Sends the command to start the given trajectory 1s from now
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

  
}
