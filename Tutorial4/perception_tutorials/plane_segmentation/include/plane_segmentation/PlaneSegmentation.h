#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
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
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <image_geometry/pinhole_camera_model.h>

// Visualization
//#include <visualization_msgs/MarkerArray.h>
//#include <visualization_msgs/Marker.h>

//#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/PoseStamped.h>

// Plane segmentation class
// computes and split the big planes from the rest of the point cloud clusters



class PlaneSegmentation
{

private:
    //! The node handle
    ros::NodeHandle nh_;
    //! Node handle in the private namespace
    ros::NodeHandle priv_nh_;


    //#>>>>TODO: Create subscriber to the PointCloud data
    // Optional: MESSAGE FILTERS COULD BE A GOOD IDEA FOR GRABBING MULTIPLE TOPICS SYNCRONIZED, NOT NEEDED THOUGH
    ros::Subscriber sub_pointCloud;

    //#>>>>TODO: Create publishers for pointclouds
    ros::Publisher pub_plane_pc_;
    ros::Publisher pub_clusters_pc_;
    // Parameters
    //#>>>>TODO: Load parameters if needed
    // Internal data
    pcl::PointCloud<pcl::PointXYZ> curr_table_pc;
    pcl::PointCloud<pcl::PointXYZ> curr_clusters_pc;  
    //#>>>>TODO: Create more containers if needed.

    //------------------ Callbacks -------------------

    // Callback for service calls
	

    //! Callback for subscribers
    //! Complete processing of new point cloud
    void processCloud(const sensor_msgs::PointCloud2ConstPtr &var); // for multiple data topics (const sensor_msgs::TypeConstPtr &var, const sensor_msgs::TypeConstPtr &var, ...)

public:
    //! Subscribes to and advertises topics
    PlaneSegmentation(ros::NodeHandle nh) : nh_(nh), priv_nh_("~") //,
        //sub(nh, "topic", 5) // constructor initialization form for the subscriber if needed
    {


        pub_plane_pc_ = nh_.advertise< pcl::PointCloud<pcl::PCLPointCloud2> >("/segmentation/plane_points", 10);
        pub_clusters_pc_ = nh_.advertise< pcl::PointCloud<pcl::PCLPointCloud2> >("/segmentation/clusters_points", 10);
       
        //#>>>>TODO: Callback function register
         //subscribe the point cloud information from the topic
        sub_pointCloud = nh_.subscribe("/hsrb/head_rgbd_sensor/depth_registered/rectified_points",10,&PlaneSegmentation::processCloud,this);
        //#>>>>TODO: Initialize params



    }

    ~PlaneSegmentation() {}
};

