// ROS headers
#include <ros/ros.h>

// MoveIt! headers
#include <moveit/move_group_interface/move_group_interface.h>

// Std C++ headers
#include <string>
#include <vector>
#include <map>

#include <exception>
#include <boost/shared_ptr.hpp>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/topic.h>
namespace jointspace
{
class jointlift
{
  public:
    void action(double (&pose)[8])
    {
      std::map<std::string, double> target_position;

      target_position["torso_lift_joint"] = pose[0];

      target_position["arm_1_joint"] = pose[1];
      target_position["arm_2_joint"] = pose[2];
      target_position["arm_3_joint"] = pose[3];
      target_position["arm_4_joint"] = pose[4];
      target_position["arm_5_joint"] = pose[5];
      target_position["arm_6_joint"] = pose[6];
      target_position["arm_7_joint"] = pose[7];

      ros::AsyncSpinner spinner(1);
      spinner.start();

      std::vector<std::string> torso_arm_joint_names;
      //select group of joints
      moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");
      //choose your preferred planner
      group_arm_torso.setPlannerId("SBLkConfigDefault");

      torso_arm_joint_names = group_arm_torso.getJoints();

      group_arm_torso.setStartStateToCurrentState();
      group_arm_torso.setMaxVelocityScalingFactor(1.0);

      for (unsigned int i = 0; i < torso_arm_joint_names.size(); ++i)
        if ( target_position.count(torso_arm_joint_names[i]) > 0 )
        {
          ROS_INFO_STREAM("\t" << torso_arm_joint_names[i] << " goal position: " << target_position[torso_arm_joint_names[i]]);
          group_arm_torso.setJointValueTarget(torso_arm_joint_names[i], target_position[torso_arm_joint_names[i]]);
        }

      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      group_arm_torso.setPlanningTime(5.0);

      ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

      // Execute the plan
      ros::Time start = ros::Time::now();

      group_arm_torso.move();

      ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());

      spinner.stop();



    }
    
  

};

}




