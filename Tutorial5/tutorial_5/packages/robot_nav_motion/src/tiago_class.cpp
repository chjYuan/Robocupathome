#include <robot_nav_motion/joint.h>

namespace tiago {

jointlift::jointlift()
{
    count_mutex = PTHREAD_MUTEX_INITIALIZER;
}

jointlift::~jointlift()
{

}
 void jointlift::action(double (&pose)[8])
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
    void jointlift::movejoint(double (&pose)[3])
    {
     geometry_msgs::PoseStamped goal_pose;
     goal_pose.header.frame_id = "base_link";
     goal_pose.pose.position.x = pose[0];//
     goal_pose.pose.position.y = pose[1];
     goal_pose.pose.position.z = pose[2];
    // goal_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(-0.011, 1.57, 0.037);
     goal_pose.pose.orientation.x=-0.714341;
     goal_pose.pose.orientation.y=-0.00450264;
     goal_pose.pose.orientation.z=-0.000369522;
     goal_pose.pose.orientation.w=0.699783;
 
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
     spinner.stop();
    }


 void navigation(float p_x,float p_y,float o_x,float o_y,float o_z,float o_w)
  { 
     typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
     MoveBaseClient ac("move_base", true);
     move_base_msgs::MoveBaseGoal goal;
     while(!ac.waitForServer(ros::Duration(5.0)))
     {
        ROS_INFO("Waiting for the move_base action server to come up");
     }
     goal.target_pose.header.frame_id = "map";
     goal.target_pose.header.stamp = ros::Time::now(); 
     //  set goal as 2D coordinate for A
     goal.target_pose.pose.position.x = p_x;// 3.62560582161;
     goal.target_pose.pose.position.y = p_y;   // -0.871481537819;
    //set goal orientation
     goal.target_pose.pose.orientation.x = o_x;//0.0;
     goal.target_pose.pose.orientation.y = o_y;//0.0;
     goal.target_pose.pose.orientation.z = o_z;//-0.808273690062;
     goal.target_pose.pose.orientation.w = o_w;//0.588806964932;
     ROS_INFO("Sending goal");
     ac.sendGoal(goal);

     ac.waitForResult();

     if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
         ROS_INFO(" Tiago moved to A");
     else
         ROS_INFO("The Tiago failed to move  for some reason");
  }

}





































void navigation(float p_x,float p_y,float o_x,float o_y,float o_z,float o_w)
  { 
     typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
     MoveBaseClient ac("move_base", true);
     move_base_msgs::MoveBaseGoal goal;
     goal.target_pose.header.frame_id = "map";
     goal.target_pose.header.stamp = ros::Time::now(); 
     //  set goal as 2D coordinate for A
     goal.target_pose.pose.position.x = p_x;// 3.62560582161;
     goal.target_pose.pose.position.y = p_y;   // -0.871481537819;
    //set goal orientation
     goal.target_pose.pose.orientation.x = o_x;//0.0;
     goal.target_pose.pose.orientation.y = o_y;//0.0;
     goal.target_pose.pose.orientation.z = o_z;//-0.808273690062;
     goal.target_pose.pose.orientation.w = o_w;//0.588806964932;
     ROS_INFO("Sending goal");
     ac.sendGoal(goal);

     ac.waitForResult();

     if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
         ROS_INFO(" Tiago moved to A");
     else
         ROS_INFO("The Tiago failed to move  for some reason");
  }