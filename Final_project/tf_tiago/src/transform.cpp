#include <ros/ros.h>
#include <tf_tiago/transform.h>

namespace transformSpace
{
    Transform::Transform(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
    {
        count_mutex = PTHREAD_MUTEX_INITIALIZER;

       /*************************************************************************************************************/
       
        /*get the goal_reach to start trajectory computation*/

       /*************************************************************************************************************/
        
        goal_res_g.reach = false; // set flag to false
        goal_reach = nh_.advertiseService("reachCheck_handle", &transformSpace::Transform::reachCallback, this);

        while (ros::ok())   // wait until tiago reaches the position
        {
            ROS_INFO("Tiago is navigating, waiting Tiago to reach the goal position");
            ros::spinOnce();
            if(goal_res_g.reach)
            break;
        }

        sub_points = nh_.subscribe("/xtion/depth_registered/points", 100, &transformSpace::Transform::processCloud, this);
        pub_plane_pc_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZ> >("/segmentation/plane_points", 10);
        pub_clusters_pc_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZ> >("/segmentation/clusters_points", 10);       
        doorshaft_client =  nh_.serviceClient<tf_tiago::doorshaft>("/doorshaft_position");
        pub_handle_pc_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZ> >("/segmentation/handle_points", 10);
        pub_firstplane_pc_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZ> >("/segmentation/firstplane", 10);
        //------------------------------------------------------------------------------------------------
        // Transform to baselink

        ros::ServiceClient tf_trans_client = nh_.serviceClient <tf_lookup::lookupTransform> ("lookupTransform");
        tf_lookup::lookupTransform tf_srv;

        //tf_lookup service client
        tf_srv.request.source_frame = "xtion_rgb_optical_frame";
        tf_srv.request.target_frame = "torso_lift_link";

        if(tf_trans_client.call(tf_srv)) // Service called
        {
        ROS_INFO("Looking up for the transform, service tf_lookup called");     
        ROS_INFO_STREAM(tf_srv.response);

        //set T
        Transition_g[0] = tf_srv.response.transform.transform.translation.x;
        Transition_g[1] = tf_srv.response.transform.transform.translation.y;
        Transition_g[2] = tf_srv.response.transform.transform.translation.z;

        //set R
        q_g.x() = tf_srv.response.transform.transform.rotation.x;
        q_g.y() = tf_srv.response.transform.transform.rotation.y;
        q_g.z() = tf_srv.response.transform.transform.rotation.z;
        q_g.w() = tf_srv.response.transform.transform.rotation.w;
            
        //Quaterniond to Rotation
        Rotation_g = q_g.toRotationMatrix();
        }

        else
        {
        ROS_ERROR_STREAM("Failed to call the service tf_lookup ");
        }
        
        handle3d_server = nh_.advertiseService("handle3d", &transformSpace::Transform::handle3d_Callback, this);


    }
    Transform::~Transform() {}

    /*************************************************************************************************************/
       
    /*process the point cloud and extract the largest plane*/

    /*************************************************************************************************************/
    void Transform::processCloud(const sensor_msgs::PointCloud2ConstPtr &var)
    {   pthread_mutex_lock( &this->count_mutex ); //!! important !!
    
    
        pcl::PointCloud< pcl::PointXYZ > pc; // internal data
        fromROSMsg(*var,pc);	//Convert the data to the internal var
	    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pc.makeShared(); // cloud to operate
	    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered( new pcl::PointCloud<pcl::PointXYZ> ); // cloud to store the filter the data
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_nonplane( new pcl::PointCloud<pcl::PointXYZ> ); 
	//Down sample the pointcloud using VoxelGrid
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud (cloud);
        sor.setLeafSize (0.01f, 0.01f, 0.01f);
        sor.filter (*cloud_filtered);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud (cloud_filtered);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (0.0, 1.0);
        pass.filter (*cloud_filtered);
        std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size()  << " data points." << std::endl;

        // Create the segmentation object for the plane model and set all the parameters using pcl::SACSegmentation<pcl::PointXYZ>
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers( new pcl::PointIndices );
        pcl::ModelCoefficients::Ptr coefficients( new pcl::ModelCoefficients );


        //set parameters of the SACS segmentation
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (1000);
        seg.setDistanceThreshold (0.01);

        int nr_points = (int) cloud_filtered->points.size();

        // Segment the planes
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
	
	//Publish biggest plane
      
        
    /*************************************************************************************************************/

        // extract the normal vector

 	/*************************************************************************************************************/

	//Create the normal estimation class, and pass the input dataset to it
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_door = curr_table_pc.makeShared();
        ne.setInputCloud (cloud_door);

        // Create an empty kdtree representation, and pass it to the normal estimation object.
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
        ne.setSearchMethod (tree);

        // Output datasets
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

        // Use all neighbors in a sphere of radius 3cm
        ne.setRadiusSearch (0.03);
        // Compute the features
        ne.compute (*cloud_normals);

        float sum_x,sum_y,sum_z,normal_x,normal_y,normal_z;
        int num = 0;
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
        //cout<<num<<endl;
        cout<<"normal_door_x "<<normal_door_x<<endl;
        cout<<"normal_door_y "<<normal_door_y<<endl;
        cout<<"normal_door_z "<<normal_door_z<<endl;

    /*************************************************************************************************************/

        // transform the point cloud

 	/*************************************************************************************************************/
        Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
        float alpha;
        // if(abs(normal_door_x)<0.05)
        // {
        //     alpha = 3.1415926/2;
        // }   
        // else
        // {
        if(normal_door_x>0.0)
        {
            alpha = atan(normal_door_y/normal_door_x); 
        }   
        else
        {
            //alpha = 3.1415926+atan(normal_door_y/normal_door_x); 
            alpha = 3.1415926/2+atan(abs(normal_door_x/normal_door_y)); 
        }
        //float beta = 3.1415926+atan(sqrt(normal_door_x*normal_door_x+normal_door_y*normal_door_y)/normal_door_z); // The angle of rotation in radians
        float beta = 3.1415926+atan(sqrt(normal_door_x*normal_door_x+normal_door_y*normal_door_y)/normal_door_z);
        transform_1 (0,0) = cos (alpha)*cos(beta);
        transform_1 (0,1) = -sin(alpha);
        transform_1 (0,2) = cos(alpha)*sin(beta);
        transform_1 (1,0) = sin (alpha)*cos(beta);
        transform_1 (1,1) = cos (alpha);
        transform_1 (1,2) = sin (alpha)*sin(beta);
        transform_1 (2,0) = -sin (beta);
        transform_1 (2,1) = 0;
        transform_1 (2,2) = cos (alpha);
        
        // Executing the transformation
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
        pcl::PointCloud<pcl::PointXYZ>::Ptr reconstructed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
        // You can either apply transform_1 or transform_2; they are the same
        pcl::transformPointCloud (*cloud_filtered, *transformed_cloud, transform_1.inverse());

	/*************************************************************************************************************/
      
	//filter the transformed cloud and reconstruct

	/*************************************************************************************************************/
        // Create the segmentation object for the plane model and set all the parameters using pcl::SACSegmentation<pcl::PointXYZ>
        pcl::SACSegmentation<pcl::PointXYZ> seg1;
        pcl::PointIndices::Ptr inliers1( new pcl::PointIndices );
        pcl::ModelCoefficients::Ptr coefficients1( new pcl::ModelCoefficients );


        //set parameters of the SACS segmentation
        // Optional
        seg1.setOptimizeCoefficients (true);
        // Mandatory
        seg1.setModelType (pcl::SACMODEL_PLANE);
        seg1.setMethodType (pcl::SAC_RANSAC);
        seg1.setMaxIterations (1000);
        seg1.setDistanceThreshold (0.01);

        int nr_points1 = (int) transformed_cloud->points.size();

        // Segment the planes
        // Create the filtering object

        pcl::ExtractIndices<pcl::PointXYZ> extract1;
        seg1.setInputCloud (transformed_cloud);
        seg1.segment (*inliers1, *coefficients1);
        extract1.setInputCloud (transformed_cloud);
        extract1.setIndices (inliers1);
        extract1.setNegative (false);
        extract1.filter (transformed_door);


        pcl::PointXYZ min;
        pcl::PointXYZ max;
        pcl::getMinMax3D(transformed_door,min,max);
        cout<<"max of door "<<max.z<<endl;
        cout<<"min of door "<<min.z<<endl;
        
        pcl::PassThrough<pcl::PointXYZ> handle_cloud;
        handle_cloud.setInputCloud (transformed_cloud);
        handle_cloud.setFilterFieldName ("z");
        handle_cloud.setFilterLimits (max.z+0.05, 0);
        handle_cloud.setNegative (false);
        handle_cloud.filter (*transformed_cloud);

        

        pcl::transformPointCloud (*transformed_cloud,*reconstructed_cloud, transform_1);

    /*************************************************************************************************************/    
	//calculate the 3D coordinate of handle
	/*************************************************************************************************************/

        
        cout<<"handle_num "<<reconstructed_cloud->points.size()<<endl;
        
        sum_x = 0;
        sum_y = 0;
        sum_z = 0;
        num=0;
        for(int i = 0;i<reconstructed_cloud->points.size();i++)
        {   
            
            if((!isnan(reconstructed_cloud->points[i].x))&&(!isnan(reconstructed_cloud->points[i].y))&&(!isnan(reconstructed_cloud->points[i].z)))
            {
                sum_x+=reconstructed_cloud->points[i].x;
                sum_y+=reconstructed_cloud->points[i].y;
                sum_z+=reconstructed_cloud->points[i].z;
                num++;
                cout<<num<<"x  "<<reconstructed_cloud->points[i].x<<endl;
                cout<<num<<"y  "<<reconstructed_cloud->points[i].y<<endl;
                cout<<num<<"z  "<<reconstructed_cloud->points[i].z<<endl;
            }

                 
            

        }
        
        float handle_x, handle_y,handle_z;
        
        handle_x = sum_x/num;
        handle_y = sum_y/num;
        handle_z = sum_z/num; 

        cout<<"handle_x "<<handle_x<<endl;
        cout<<"handle_y "<<handle_y<<endl;
        cout<<"handle_z "<<handle_z<<endl;
        pthread_mutex_unlock( &this->count_mutex );

    
    /*************************************************************************************************************/
      
	//transform from optical frame to do the torso lift frame

	/*************************************************************************************************************/
	    
        coordinate_normal_g.x = normal_door_x;
        coordinate_normal_g.y = normal_door_y;
        coordinate_normal_g.z = normal_door_z;
        coordinate_handle_g.x = handle_x;
        coordinate_handle_g.y = handle_y;
        coordinate_handle_g.z = handle_z;
        
        Transform::transformtoBaselink ();
        
        double normal_torso_x,normal_torso_y,normal_torso_z;

        handle_torso_x = coordinate_handle_base_g.x;
        handle_torso_y = coordinate_handle_base_g.y;
        handle_torso_z = coordinate_handle_base_g.z;
        normal_torso_x = coordinate_normal_base_g.x;
        normal_torso_y = coordinate_normal_base_g.y;
        normal_torso_z = coordinate_normal_base_g.z;

	//calculate the required data
        doorshaft_x = handle_torso_x-0.038*normal_torso_x+0.508*normal_torso_y;
        doorshaft_y = handle_torso_y-0.038*normal_torso_y-0.508*normal_torso_x;
        doorshaft_z = handle_torso_z;
        theta = -atan(normal_torso_y/normal_torso_x);

        cout<<"handle_torso_x"<<handle_torso_x<<endl;
        cout<<"handle_torso_y"<<handle_torso_y<<endl;
        cout<<"handle_torso_z"<<handle_torso_z<<endl;
        cout<<"doorshaft_x"<<doorshaft_x<<endl;
        cout<<"doorshaft_y"<<doorshaft_y<<endl;
        cout<<"doorshaft_z"<<doorshaft_z<<endl;
        cout<<"theta"<<theta<<endl;


        sensor_msgs::PointCloud2 msg_table;
        pcl::toROSMsg(transformed_door,msg_table);
        pub_plane_pc_.publish(msg_table);

        sensor_msgs::PointCloud2 msg_cluster;
        pcl::toROSMsg(*transformed_cloud,msg_cluster);
        pub_clusters_pc_.publish(msg_cluster);

        sensor_msgs::PointCloud2 msg_firstplane;
        pcl::toROSMsg(curr_table_pc,msg_firstplane);
        pub_firstplane_pc_.publish(msg_firstplane);

        sensor_msgs::PointCloud2 msg_handle;
        pcl::toROSMsg(*reconstructed_cloud,msg_handle);
        pub_handle_pc_.publish(msg_handle);    
 
    }

    // ----------------transform function--------------------------
    void Transform::transformtoBaselink ()
    {
        Vector3d coordinate_normal_vec; //vector3d pose of optical
        Vector3d coordinate_normal_base;  //vector3d pose of the base 
        Vector3d coordinate_handle_vec; //vector3d pose of optical
        Vector3d coordinate_handle_base;  //vector3d pose of the base 
        
        //ROS_INFO("coordinate transform running");

        if((!isnan(coordinate_normal_g.z))&&(!isnan(coordinate_handle_g.z)))
        {
            coordinate_normal_vec[0] = coordinate_normal_g.x;
            coordinate_normal_vec[1] = coordinate_normal_g.y;
            coordinate_normal_vec[2] = coordinate_normal_g.z;  //geometry_msgs::point to vector3d
            coordinate_normal_base = Rotation_g * coordinate_normal_vec;
            coordinate_normal_base_g.x = coordinate_normal_base[0];
            coordinate_normal_base_g.y = coordinate_normal_base[1];
            coordinate_normal_base_g.z = coordinate_normal_base[2];

            coordinate_handle_vec[0] = coordinate_handle_g.x;
            coordinate_handle_vec[1] = coordinate_handle_g.y;
            coordinate_handle_vec[2] = coordinate_handle_g.z;  //geometry_msgs::point to vector3d
            coordinate_handle_base = Rotation_g * coordinate_handle_vec + Transition_g;  // Transform: RT
            coordinate_handle_base_g.x = coordinate_handle_base[0];
            coordinate_handle_base_g.y = coordinate_handle_base[1];
            coordinate_handle_base_g.z = coordinate_handle_base[2];

            ROS_INFO("------------------------------------");
            
        }
        
    }

    bool Transform::reachCallback (tf_tiago::ReachGoal::Request& req, tf_tiago::ReachGoal::Response& res)
    {
            pthread_mutex_lock( &this->count_mutex ); //!! important !!
            ROS_INFO("------------------------------------------");
            ROS_INFO("The Tiago has reached the goal position !!!");
            ROS_INFO("------------------------------------------");
            goal_res_g.reach = true;
            pthread_mutex_unlock( &this->count_mutex );
        
            return true;
  }
  bool Transform::handle3d_Callback(tf_tiago::doorshaft::Request& req, tf_tiago::doorshaft::Response& res)
    {
	   ROS_INFO("handle3d service called, sending the target position of base_link");
    	   
           
           res.doorshaft_res.x = doorshaft_x;
    	   res.doorshaft_res.y = doorshaft_y;
    	   res.doorshaft_res.z = doorshaft_z;
	       res.theta_res = theta;
           res.handle_res.x = handle_torso_x;
	       res.handle_res.y = handle_torso_y;
           res.handle_res.z = handle_torso_z;

           return true;
   }

}
