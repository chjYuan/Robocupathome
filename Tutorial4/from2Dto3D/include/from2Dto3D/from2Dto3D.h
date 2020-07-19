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

//#include <image_geometry/pinhole_camera_model.h>

#include <perception_msgs/Rect.h>


//#>>>>TODO: FIX DEPENDENCIES IN CMakeLists.txt and package.xml (Make sure that everithing compiles in one shot.)

//#>>>>TODO: Separate this template in a class library (.h and .cpp files) and an node (.cpp file). The header files must be in a "include" folder in the package.


using namespace std;
using namespace cv;


class From2Dto3D
{

    private:
      // The node handle
      ros::NodeHandle nh_;
      // Node handle in the private namespace
      ros::NodeHandle priv_nh_;

      //#>>>>TODO: Define publishers and subscribers
      ros::Publisher pub;
      ros::Subscriber sub_1;
      ros::Subscriber sub_2;
      

      int obj_center_x;
      int obj_center_y;
      string obj_frameid;

      //#>>>>TODO: Define the pointcloud structure and the bounding box local copy

      pcl::PointCloud <pcl::PointXYZRGB> obj_cloud;
      geometry_msgs::PointStamped obj_pose;



      // A tf transform listener if needed
      tf::TransformListener listener_;


      //------------------ Callbacks -------------------

      // Process clusters
      void processCloud(const sensor_msgs::PointCloud2ConstPtr& pc);
      // Process bounding boxes
      void processRect(const perception_msgs::RectConstPtr& r);


    public:
      // Subscribes to and advertises topics
      From2Dto3D(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
      {

        //#>>>>TODO: Set publishers and subscribers.

        // subscribers to the bounding boxes and the point cloud
        // format:
        // sub_name = nh_.subscribe<Type>("topic", queuesize, Function_of_the_class, this);
        sub_1 = nh_.subscribe<perception_msgs::Rect>("/object_recognition", 100, &From2Dto3D::processRect, this);
        sub_2 = nh_.subscribe<sensor_msgs::PointCloud2>("/hsrb/head_rgbd_sensor/depth_registered/rectified_points", 100, &From2Dto3D::processCloud, this);
        

        // Publishers
        // format:
        //pub_name = nh_.advertise< Type >("topic", queuesize);
        pub = nh_.advertise<geometry_msgs::PointStamped> ("object_position", 100);

        ROS_INFO("from2Dto3D initialized ...");

      }

      ~From2Dto3D() {}
};