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
#include <pcl/common/common.h>
#include <image_geometry/pinhole_camera_model.h>
#include <pcl/visualization/pcl_visualizer.h>
// Visualization
//#include <visualization_msgs/MarkerArray.h>
//#include <visualization_msgs/Marker.h>


//#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/PoseStamped.h>

#include <plane_segmentation/PlaneSegmentation.h>


using namespace std;
using namespace cv;

//#>>>>TODO: FIX DEPENDENCIES IN CMakeLists.txt and package.xml (Make sure that everithing compiles in one shot.)

//#>>>>TODO: Separate this template in a class library (.h and .cpp files) and an node (.cpp file). The header files must be in a "include" folder in the package.



void planSegmentationSpace::PlaneSegmentation::processCloud(const sensor_msgs::PointCloud2ConstPtr &var)
{

    pcl::PointCloud< pcl::PointXYZ > pc; // internal data
       
	//#>>>>TODO: Convert the data to the internal var (pc) using pcl function: fromROSMsg

    fromROSMsg(*var,pc);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pc.makeShared(); // cloud to operate

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered( new pcl::PointCloud<pcl::PointXYZ> ); // cloud to store the filter the data

    std::cout << "PointCloud before filtering has: " << pc.points.size() << " data points." << std::endl; //*
    std::cout << "width: " << pc.width << "height: " << pc.height << std::endl;


    //#>>>>TODO: Down sample the pointcloud using VoxelGrid
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*cloud_filtered);

    //----
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud_filtered);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 1.0);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_filtered);
    std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size()  << " data points." << std::endl;
    


    // Create the segmentation object for the plane model and set all the parameters using pcl::SACSegmentation<pcl::PointXYZ>
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers( new pcl::PointIndices );
    pcl::ModelCoefficients::Ptr coefficients( new pcl::ModelCoefficients );
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane( new pcl::PointCloud<pcl::PointXYZ>() );

    //#>>>>TODO: set parameters of the SACS segmentation
	// Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.01);

    //----

    int nr_points = (int) cloud_filtered->points.size();

    // Segment the planes using pcl::SACSegmentation::segment() function and pcl::ExtractIndices::filter() function
	// TODO
        // If you want to extract more than one plane you have to do a while
    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (curr_table_pc);
    extract.setNegative (true);
    extract.filter (curr_clusters_pc);

    pcl::PointXYZ min;
    pcl::PointXYZ max;
    pcl::getMinMax3D(curr_table_pc,min,max);
    //cout<<"max of table "<<curr_table_pc.points[0].x<<endl;
    //cout<<"max of door "<<max<<endl;
    //cout<<"min of door "<<min<<endl;
    float sum_x,sum_y,sum_z,normal_x,normal_y,normal_z;
    sum_x = 0;
    sum_y = 0;
    sum_z = 0;
    int num = 0;
    
    float handle_x, handle_y,handle_z;
    handle_x = sum_x/num;
    handle_y = sum_y/num;
    handle_z = sum_z/num;
    cout<<"handle_x "<<handle_x<<endl;
    cout<<"handle_y "<<handle_y<<endl;
    cout<<"handle_z "<<handle_z<<endl;
    //----

    
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_door = curr_table_pc.makeShared();
    ne.setInputCloud (cloud_door);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (0.03);
    // Compute the features
    ne.compute (*cloud_normals);

    sum_x = 0;
    sum_y = 0;
    sum_z = 0;
    num = 0;
    for(int i = 0;i<cloud_normals->points.size();i++)
    {
	sum_x+=cloud_normals->points[i].normal_x;
	sum_y+=cloud_normals->points[i].normal_y;
	sum_z+=cloud_normals->points[i].normal_z;
        num++;
    }
    float normal_door_x, normal_door_y,normal_door_z;
    normal_door_x = sum_x/num;
    normal_door_y = sum_y/num;
    normal_door_z = sum_z/num;
    //cout<<"cloud_normals_size "<<cloud_normals->points.size()<<endl;
    //cout<<"cloud_table_size "<<curr_table_pc.points.size()<<endl;
    cout<<num<<endl;
    cout<<"normal_door_x "<<normal_door_x<<endl;
    cout<<"normal_door_y "<<normal_door_y<<endl;
    cout<<"normal_door_z "<<normal_door_z<<endl;
    
    //#>>>>TODO: Publish biggest plane

	// Tips: 
	// - you can copy the pointcloud using cl::copyPointCloud()
	// - Set the header frame_id to the pc2 header frame_id
	// - you can use pcl_conversions::toPCL() to convert the stamp from pc2 header to pointcloud header stamp
	// - to publish -> pub_plane_pc_.publish( pointcloud_variable.makeShared() )
    //----
    sensor_msgs::PointCloud2 msg_table;
    pcl::toROSMsg(curr_table_pc,msg_table);
    pub_plane_pc_.publish(msg_table);

    //#>>>>TODO: Publish other clusters
    // Tip: similar to the previous publish
    sensor_msgs::PointCloud2 msg_cluster;
    pcl::toROSMsg(curr_clusters_pc,msg_cluster);
    pub_clusters_pc_.publish(msg_cluster);


    return;


}




