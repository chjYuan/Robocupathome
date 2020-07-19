#include "ros/ros.h"
#include "object_detection/ObjectRecognition.h"

void recognitionspace::Objectdetection::processRecognation(const darknet_ros_msgs::BoundingBoxes::ConstPtr &var)
{
    perception_msgs::RectArray msgs;
    perception_msgs::Rect msg;

    
    for(int i=0; i<var->bounding_boxes.size();i++)
    {
      msg.header.frame_id=var->bounding_boxes[i].Class;
      msg.id = var->bounding_boxes[i].id;
      msg.x = var->bounding_boxes[i].xmin;
      msg.y = var->bounding_boxes[i].ymin;
      msg.height = var->bounding_boxes[i].ymax-var->bounding_boxes[i].ymin;
      msg.width = var->bounding_boxes[i].xmax-var->bounding_boxes[i].xmin;
      msgs.rect_array.push_back(msg);
    }
    pub.publish(msgs);
    ROS_INFO_STREAM(msgs);
    msgs.rect_array.clear();
    


}
