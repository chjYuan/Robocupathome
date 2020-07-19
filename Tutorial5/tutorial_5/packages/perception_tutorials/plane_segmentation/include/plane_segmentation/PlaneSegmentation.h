#ifndef PLANSEGMENTATIONCLASS_H
#define PLANSEGMENTATIONCLASS_H

/*********************************************************************
* ROS INCLUDES
********************************************************************/
#include <ros/ros.h>

#include <std_msgs/String.h>


/*********************************************************************
* EIGEN INCLUDES
********************************************************************/
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <Eigen/Core>

using namespace Eigen;

namespace planSegmentationSpace
{
class PlaneSegmentation
{

    private:
    //! The node handle
    ros::NodeHandle nh_;
    //! Node handle in the private namespace
    ros::NodeHandle priv_nh_;


    //#>>>>TODO: Create subscriber to the PointCloud data
    // Optional: MESSAGE FILTERS COULD BE A GOOD IDEA FOR GRABBING MULTIPLE TOPICS SYNCRONIZED, NOT NEEDED THOUGH
    ros::Subscriber sub;

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

        sub = nh_.subscribe("/xtion/depth_registered/points", 100, &planSegmentationSpace::PlaneSegmentation::processCloud, this);

         
        pub_plane_pc_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZ> >("/segmentation/plane_points", 10);
        pub_clusters_pc_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZ> >("/segmentation/clusters_points", 10);

        //#>>>>TODO: Callback function register
        
        //#>>>>TODO: Initialize params



    }

    ~PlaneSegmentation() {}
};


}

#endif // PLANSEGMENTATIONCLASS_H
