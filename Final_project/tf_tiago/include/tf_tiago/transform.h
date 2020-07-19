//ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Char.h>
#include <geometry_msgs/Point.h>

//Eigen
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <Eigen/Core>

//tiago tf
#include <tf_lookup/lookupTransform.h>
#include <geometry_msgs/TransformStamped.h>

//segmentation
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

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
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <image_geometry/pinhole_camera_model.h>

//#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf_tiago/doorshaft.h>
#include <tf_tiago/ReachGoal.h>


using namespace std;
using namespace Eigen;

namespace transformSpace
{
    class Transform
    {
        private:
            ros::NodeHandle nh_;
            ros::NodeHandle priv_nh_;

                
            // Optional: MESSAGE FILTERS COULD BE A GOOD IDEA FOR GRABBING MULTIPLE TOPICS SYNCRONIZED, NOT NEEDED THOUGH
            ros::Subscriber sub_points;


            ros::Publisher pub_plane_pc_;
            ros::Publisher pub_clusters_pc_;
            ros::Publisher pub_handle_pc_;
            ros::Publisher pub_firstplane_pc_;
            ros::ServiceClient doorshaft_client;

            ros::ServiceServer goal_reach;  // Service server to check if the tiago ends navgation and reaches the goal position
            ros::ServiceServer handle3d_server;

            // Internal data
            pcl::PointCloud<pcl::PointXYZ> curr_table_pc;
            pcl::PointCloud<pcl::PointXYZ> curr_clusters_pc;
            pcl::PointCloud<pcl::PointXYZ> door_filtered;
            pcl::PointCloud<pcl::PointXYZ> transformed_door;
            //! Callback for subscribers
            //! Complete processing of new point cloud           
            void processCloud(const sensor_msgs::PointCloud2ConstPtr &var); // for multiple data topics (const sensor_msgs::TypeConstPtr &var, const sensor_msgs::TypeConstPtr &var, ...)


        public:

            // store the global 3d pose in head frame   
            geometry_msgs::Point coordinate_normal_g; 
            geometry_msgs::Point coordinate_handle_g; 

            // store the global target position in baselink frame
            geometry_msgs::Point coordinate_normal_base_g; 
            geometry_msgs::Point coordinate_handle_base_g; 

            // Transform: T, R, Quaterniond
            Vector3d Transition_g;
            Quaterniond q_g;
            Matrix3d Rotation_g;

            // thread
            pthread_mutex_t count_mutex; 

            // bool flag to check if tiago reaches goal
            tf_tiago::ReachGoal::Response goal_res_g;
            //message used for handle3d
            double doorshaft_x,doorshaft_y,doorshaft_z,theta,handle_torso_x,handle_torso_y,handle_torso_z;
            

            // transform function
            void transformtoBaselink ();

            // callback function for service  "goal_reach" to check if the tiago reach the goal position
            bool reachCallback (tf_tiago::ReachGoal::Request& req, tf_tiago::ReachGoal::Response& res);
            
            // callback function for service  "handle3d" to get handle3d
            bool handle3d_Callback(tf_tiago::doorshaft::Request& req, tf_tiago::doorshaft::Response& res);

             Transform(ros::NodeHandle nh); // declaration of construct function
            ~Transform();                  // declaration of deconstruct function




    };
}
