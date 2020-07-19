// c++
#include <math.h>
#include <string>
#include <vector>
#include <iostream>
#include <pthread.h>
#include <thread>
#include <chrono>

// ROS
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>

// OpenCv
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <cv_bridge/cv_bridge.h>

// darknet_ros_msgs
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/ObjectCount.h>
#include <darknet_ros_msgs/CheckForObjectsAction.h>

#include <perception_msgs/Rect.h>

perception_msgs::Rect obj_recg_1;
perception_msgs::Rect obj_recg_2;


void recgCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg)
{  

 for(int i=0;i < msg->bounding_boxes.size();i++)
 {
   if(msg->bounding_boxes[i].Class == "bottle")
   {
      obj_recg_1.header.frame_id = msg->bounding_boxes[i].Class;
      obj_recg_1.x = msg->bounding_boxes[i].xmin;
      obj_recg_1.y = msg->bounding_boxes[i].ymin;
      obj_recg_1.width = msg->bounding_boxes[i].ymax - msg->bounding_boxes[i].ymin;
      obj_recg_1.height = msg->bounding_boxes[i].xmax - msg->bounding_boxes[i].xmin;
   }
   if(msg->bounding_boxes[i].Class == "book")
   {
      obj_recg_2.header.frame_id = msg->bounding_boxes[i].Class;
      obj_recg_2.x = msg->bounding_boxes[i].xmin;
      obj_recg_2.y = msg->bounding_boxes[i].ymin;
      obj_recg_2.width = msg->bounding_boxes[i].ymax - msg->bounding_boxes[i].ymin;
      obj_recg_2.height = msg->bounding_boxes[i].xmax - msg->bounding_boxes[i].xmin;

   }
   
 }

}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "object_recognition_node");
  
  ROS_INFO_STREAM("Object_recognition_node starting");
    
  ros::NodeHandle n;
  ros::Rate r(60);
  
  ros::Subscriber sub=n.subscribe("darknet_ros/bounding_boxes", 100, recgCallback);
  
  ros::Publisher pub_1=n.advertise<perception_msgs::Rect>("/object_recognition", 100);
  ros::Publisher pub_2=n.advertise<perception_msgs::Rect>("/object_recognition", 100);
  
  while(ros::ok())
  {
     pub_1.publish(obj_recg_1);
     pub_2.publish(obj_recg_2);
     
     ros::spinOnce();
     r.sleep();
  }
  
  

  
  

  
  return 0;


}
