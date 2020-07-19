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



namespace tiago
{
      
      ros::Publisher pub;
      void navigation(float p_x,float p_y,float o_x,float o_y,float o_z,float o_w);
      float x_3d;
      float y_3d;
      float z_3d;
      class jointlift
      {
          public:
                pthread_mutex_t count_mutex;
                jointlift();
                ~jointlift();
                void action(double (&pose)[8]);
                void movejoint(double (&pose)[3]);
   
      };
    
  
}




