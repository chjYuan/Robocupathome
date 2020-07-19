#include <ros/ros.h>
#include <from2Dto3D/from2Dto3D.h>
namespace from2Dto3DSpace
{
  From2Dto3D::From2Dto3D(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")   // definition of construct function, equal to From2Dto3D(ros::NodeHandle nh, nh_=nh, priv_nh_=nh) ?
  {
    count_mutex = PTHREAD_MUTEX_INITIALIZER; //!!! very important, if not, the value in while(ros::ok) can not change
    goal_res_g.reach = false; // set flag to false

    // Service server to check if tiago reaches the goal position
    goal_reach = nh_.advertiseService("ReachCheck", &from2Dto3DSpace::From2Dto3D::reachCallback, this);

    while (ros::ok())   // wait until tiago reaches the position
    {
      ROS_INFO("Tiago is navigating, waiting Tiago to reach the goal position");
      ros::spinOnce();
      if(goal_res_g.reach)
      break;

    }

    // subscribers to the bounding boxes and the point cloud
    sub_bounding = nh_.subscribe("rectInfo", 30, &from2Dto3DSpace::From2Dto3D::processRect, this);
    sub_cloud = nh_.subscribe("/xtion/depth_registered/points", 30, &from2Dto3DSpace::From2Dto3D::processCloud, this);

    // Publishers
    pub_3D = nh_.advertise< geometry_msgs::Point >("segmentation/point3D", 30); //pose of frame optical
    pub_base = nh_.advertise< geometry_msgs::Point > ("base_point3D", 30);      //pose of frame base_link

    //tf_lookup service client
    ros::ServiceClient tf_trans_client = nh_.serviceClient <tf_lookup::lookupTransform> ("lookupTransform");
    tf_lookup::lookupTransform tf_srv;

    tf_srv.request.source_frame = "xtion_rgb_optical_frame";
    tf_srv.request.target_frame = "base_link";

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

    //service server to send pose in base_link
    point_server = nh_.advertiseService("PoseBaselink", &from2Dto3DSpace::From2Dto3D::pointCallback, this);

    ROS_INFO("from2Dto3D initialized ...");
  }

  From2Dto3D::~From2Dto3D() {}



  void From2Dto3D::processCloud(const sensor_msgs::PointCloud2ConstPtr& pc)
  {   
    pcl::fromROSMsg(*pc,pCloud);
    geometry_msgs::Point coordinate_3d;
    string object = "bottle";
    //ros::param::get("objectname",object);
    ROS_INFO_STREAM("target is: " << object);
    
    float px,py;

    if (!isnan(boundoryCopy.rect_array[0].id))
    {      
      for(int i=0;i<boundoryCopy.rect_array.size();i++)
      {
        if(boundoryCopy.rect_array[i].header.frame_id==object)
        {
            px=boundoryCopy.rect_array[i].x+0.5*boundoryCopy.rect_array[i].width;
            py=boundoryCopy.rect_array[i].y+0.5*boundoryCopy.rect_array[i].height;
            coordinate_3d.x=pCloud.at(px,py).x;
            coordinate_3d.y=pCloud.at(px,py).y;
            coordinate_3d.z=pCloud.at(px,py).z;

            for (int j=0;j<100;j++)
            {           
                coordinate_3d.x=pCloud.at(px,py).x;
                coordinate_3d.y=pCloud.at(px,py).y;
                coordinate_3d.z=pCloud.at(px,py).z;
                if (!isnan(coordinate_3d.x))
                  {
                    pub_3D.publish(coordinate_3d);

                    //ROS_INFO_STREAM("This is a "<<object<<"");
                    //ROS_INFO_STREAM(coordinate_3d);
                    
                    // global
                    coordinate_3d_g.x = coordinate_3d.x;
                    coordinate_3d_g.y = coordinate_3d.y;
                    coordinate_3d_g.z = coordinate_3d.z;

                    j=101;
                    i=1+boundoryCopy.rect_array.size();
                  }
                    px=px+0.01;
            }
        }
      }                   
        
    }
    From2Dto3D::transformtoBaselink(); // run the tf function
    
    boundoryCopy.rect_array.clear();
  }

  void From2Dto3D::processRect(const perception_msgs::RectArrayConstPtr& r)
  {
    // tip: take a look at the pcl::PointXYZRGB structure
    for(int i=0;i<r->rect_array.size();i++)
    {
      	boundoryCopy.rect_array.push_back(r->rect_array[i]);        
    }
  }

  // transform function
  void From2Dto3D::transformtoBaselink ()
  {
    Vector3d coordinate_3d_vec; //vector3d pose of optical
    Vector3d coordinate_base;  //vector3d pose of the base 
       
    //ROS_INFO("coordinate transform running");

    if(!isnan(coordinate_3d_g.z))
    {
      coordinate_3d_vec[0] = coordinate_3d_g.x;
      coordinate_3d_vec[1] = coordinate_3d_g.y;
      coordinate_3d_vec[2] = coordinate_3d_g.z;  //geometry_msgs::point to vector3d
    
      coordinate_base = Rotation_g * coordinate_3d_vec + Transition_g;  // Transform: RT
    
      coordinate_base_g.x = coordinate_base[0];
      coordinate_base_g.y = coordinate_base[1];
      coordinate_base_g.z = coordinate_base[2];

      ROS_INFO("------------------------------------");
      ROS_INFO("The object position according to base_link is: ");
      ROS_INFO_STREAM(coordinate_base_g.x);
      ROS_INFO_STREAM(coordinate_base_g.y);
      ROS_INFO_STREAM(coordinate_base_g.z);
     }

  }

  // srv PointBase Callback function
  bool From2Dto3D::pointCallback (from2Dto3D::PointBase::Request& req, from2Dto3D::PointBase::Response& res)
  {
    ROS_INFO("PointBase service called, sending the target position of base_link");

    res.pose.x = coordinate_base_g.x;
    res.pose.y = coordinate_base_g.y;
    res.pose.z = coordinate_base_g.z;

    return true;
  }

  //srv ReachGoal Callback function
  bool From2Dto3D::reachCallback (from2Dto3D::ReachGoal::Request& req, from2Dto3D::ReachGoal::Response& res)
  {
    pthread_mutex_lock( &this->count_mutex ); //!! important !!
    ROS_INFO("------------------------------------------");
    ROS_INFO("The Tiago has reached the goal position !!!");
    ROS_INFO("------------------------------------------");
    goal_res_g.reach = true;
    pthread_mutex_unlock( &this->count_mutex );
  
    return true;
  }

}





