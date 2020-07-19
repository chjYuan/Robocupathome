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

#include <image_geometry/pinhole_camera_model.h>

// Visualization
//#include <visualization_msgs/MarkerArray.h>
//#include <visualization_msgs/Marker.h>

//#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/PoseStamped.h>

#include <plane_segmentation/PlaneSegmentation.h>


using namespace std;
using namespace cv;
using namespace planSegmentationSpace;

//! Callback for processing the Point Cloud data




int main(int argc, char** argv)
{
    ros::init(argc, argv, "plane_segmentation");
    ros::NodeHandle nh;
    PlaneSegmentation node(nh);
    ros::spin();
    return 0;
}
