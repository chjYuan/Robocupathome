#include <from2Dto3D/from2Dto3D.h>

void From2Dto3D::processCloud(const sensor_msgs::PointCloud2ConstPtr& pc)
{    
    ROS_INFO("Waiting for Rect msg");
    pcl::fromROSMsg(*pc,obj_cloud); // change ROS message PointCloud2 to pcl message PointXYZRGB

}

void From2Dto3D::processRect(const perception_msgs::RectConstPtr& r)
{
//#>>>>TODO: process bounding box and send 3D position to the topic
// tip: take a look at the pcl::PointXYZRGB structure
obj_center_x = r->x + 0.5*r->width;
obj_center_y = r->y + 0.5*r->height;

ROS_INFO("from2Dto3D running!");


obj_pose.point.x = obj_cloud.at(obj_center_x,obj_center_y).x;
obj_pose.point.y = obj_cloud.at(obj_center_x,obj_center_y).y;
obj_pose.point.z = obj_cloud.at(obj_center_x,obj_center_y).z;


obj_pose.header.frame_id = r->header.frame_id;
pub.publish(obj_pose);
ROS_INFO("Object position is: ");
ROS_INFO_STREAM(obj_pose.header.frame_id);

ROS_INFO_STREAM("x = " << obj_pose.point.x);
ROS_INFO_STREAM("y = " << obj_pose.point.y);
ROS_INFO_STREAM("z = " << obj_pose.point.z);
ROS_INFO_STREAM("------------------------------------");
}