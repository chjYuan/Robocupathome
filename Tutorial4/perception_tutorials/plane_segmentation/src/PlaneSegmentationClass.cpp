#include <plane_segmentation/PlaneSegmentation.h>


//! Callback for processing the Point Cloud data
void PlaneSegmentation::processCloud(const sensor_msgs::PointCloud2ConstPtr &var)
{

    pcl::PointCloud< pcl::PointXYZ > pc; // internal data
    

   
	//#>>>>TODO: Convert the data to the internal var (pc) using pcl function: fromROSMsg

    pcl::fromROSMsg(*var,pc);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pc.makeShared(); // cloud to operate
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered( new pcl::PointCloud<pcl::PointXYZ> ),cloud_f (new pcl::PointCloud<pcl::PointXYZ>); // cloud to store the filter the data
    
    std::cout << "PointCloud before filtering has: " << pc.points.size() << " data points." << std::endl; //*
    std::cout << "width: " << pc.width << "height: " << pc.height << std::endl;


    //#>>>>TODO: Down sample the pointcloud using VoxelGrid
   
    /* set the orignal point cloud to a grid form to reduce number of points */
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*cloud_filtered);
    //----

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
    seg.setDistanceThreshold (0.02);
    seg.setMaxIterations (1000);
    

    //----

    int nr_points = (int) cloud_filtered->points.size();

    // Segment the planes using pcl::SACSegmentation::segment() function and pcl::ExtractIndices::filter() function
	// TODO
        // If you want to extract more than one plane you have to do a while
    pcl::ExtractIndices<pcl::PointXYZ> extract;
  
    /************************************************************************************** */ 

      /*pass filter:filtering the outliers on z and y axis  */

    /************************************************************************************** */  
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud (cloud_filtered);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (0.0, 1.0);
        //pass.setFilterLimitsNegative (true);
        pass.filter (*cloud_filtered);


        pcl::PassThrough<pcl::PointXYZ> pass1;
        pass1.setInputCloud (cloud_filtered);
        pass1.setFilterFieldName ("y");
        pass1.setFilterLimits (-0.2, 0.17);
        //pass.setFilterLimitsNegative (true);
        pass1.filter (*cloud_filtered);
    /************************************************************************************** */    
        
        /* Segment the largest planar component from the remaining cloud */
        /* segement the plane part */
        /* extract the object part */

    /************************************************************************************** */ 
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);
         // Extract the inliers
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_plane);
        
        // Create the filtering object
        extract.setNegative (true);
        extract.filter (*cloud_f);


    //#>>>>TODO: Publish biggest plane

	// Tips: 
	// - you can copy the pointcloud using pcl::copyPointCloud()
	// - Set the header frame_id to the pc2 header frame_id
	// - you can use pcl_conversions::toPCL() to convert the stamp from pc2 header to pointcloud header stamp
	// - to publish -> pub_plane_pc_.publish( pointcloud_variable.makeShared() )
    //----
    pcl::copyPointCloud(*cloud_plane,curr_table_pc);
    sensor_msgs::PointCloud2 out_plane;
    pcl::toROSMsg(curr_table_pc,out_plane);
    pub_plane_pc_.publish(out_plane); 
    //#>>>>TODO: Publish other clusters
    // Tip: similar to the previous publish
    pcl::copyPointCloud(*cloud_f,curr_clusters_pc);
    sensor_msgs::PointCloud2 out_object;
    pcl::toROSMsg(curr_clusters_pc,out_object);
    pub_clusters_pc_.publish(out_object); 
    


    return;


}