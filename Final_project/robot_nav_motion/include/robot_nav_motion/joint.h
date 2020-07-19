// ROS headers
#include <ros/ros.h>

// MoveIt! headers
#include <moveit/move_group_interface/move_group_interface.h>
#include <move_base_msgs/MoveBaseAction.h>
// Std C++ headers
#include <string>
#include <vector>
#include <map>
#include <from2Dto3D/PointBase.h>
#include <from2Dto3D/ReachGoal.h>
#include <tf_tiago/doorshaft.h>
#include <tf_tiago/ReachGoal.h>
#include <ikTiago/gripper.h>
#include <ikTiago/doorinfo.h>

namespace tiago
{
      
      ros::Publisher pub;
      void navigation(float p_x,float p_y,float o_x,float o_y,float o_z,float o_w);
      float x_3d,y_3d,z_3d;
      double doorshaft_x,doorshaft_y,doorshaft_z,theta,handle_x,handle_y,handle_z;
      const double PI=3.1415;
      class jointlift
      {
          public:
                pthread_mutex_t count_mutex;
                jointlift();
                ~jointlift();
                void action(double (&pose)[8]);
                void movejoint(double (&pose)[3]);
                void movejoint_1(double (&pose)[6]);
   
      };
    
  
}




