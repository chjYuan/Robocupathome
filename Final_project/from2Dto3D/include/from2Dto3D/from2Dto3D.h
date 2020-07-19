#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Char.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


// PCL specific includes
#include <pcl_ros/point_cloud.h> // enable pcl publishing
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>

#include <image_geometry/pinhole_camera_model.h>

#include <perception_msgs/Rect.h>
#include <perception_msgs/RectArray.h>
#include <math.h>

//Eigen
#include <tf/transform_listener.h>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <Eigen/Core>

//tiago tf
#include <tf_lookup/lookupTransform.h>
#include <geometry_msgs/TransformStamped.h>

//srv
#include <from2Dto3D/PointBase.h>
#include <from2Dto3D/ReachGoal.h>







using namespace std;
using namespace cv;
using namespace Eigen;

namespace from2Dto3DSpace
{
class From2Dto3D
{

    private:
      // The node handle
      ros::NodeHandle nh_;
      // Node handle in the private namespace
      ros::NodeHandle priv_nh_;

      //Define publishers and subscribers
      ros::Subscriber sub_bounding;
      ros::Subscriber sub_cloud;
      ros::Publisher pub_3D;
      ros::Publisher pub_base; //publish the 3d coordinate in base_link

      ros::ServiceServer point_server;  // Service server to send the target pose in baselink
      ros::ServiceServer goal_reach;  // Service server to check if the tiago ends navgation and reaches the goal position

      

      //Define the pointcloud structure and the bounding box local copy
      pcl::PointCloud<pcl::PointXYZRGB> pCloud;
      perception_msgs::RectArray boundoryCopy;

      //cluster callbacks 
      // Process clusters
      void processCloud(const sensor_msgs::PointCloud2ConstPtr& pc);
      // Process bounding boxes
      void processRect(const perception_msgs::RectArrayConstPtr & r);


    public:
      // store the global 3d pose in head frame   
      geometry_msgs::Point coordinate_3d_g; 

      // store the global target position in baselink frame
      geometry_msgs::Point coordinate_base_g; 

      // Transform: T, R, Quaterniond
      Vector3d Transition_g;
      Quaterniond q_g;
      Matrix3d Rotation_g;

      // bool flag to check if tiago reaches goal
      from2Dto3D::ReachGoal::Response goal_res_g; 
      // thread
      pthread_mutex_t count_mutex; 


      // transform function
      void transformtoBaselink ();
      
      // callback function for service "point_server" to send the target pose in baselink
      bool pointCallback (from2Dto3D::PointBase::Request& req, from2Dto3D::PointBase::Response& res);

      // callback function for service  "goal_reach" to check if the tiago reach the goal position
      bool reachCallback (from2Dto3D::ReachGoal::Request& req, from2Dto3D::ReachGoal::Response& res);

      From2Dto3D(ros::NodeHandle nh); // declaration of construct function
      ~From2Dto3D();                  // declaration of deconstruct function

};
}
