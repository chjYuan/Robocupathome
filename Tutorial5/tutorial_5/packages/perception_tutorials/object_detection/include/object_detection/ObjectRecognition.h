#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "perception_msgs/Rect.h"
#include "perception_msgs/RectArray.h"
namespace recognitionspace
{
class Objectdetection
{
private:
 ros::NodeHandle nh_;
 ros::NodeHandle priv_nh_;
 ros::Subscriber sub;
 ros::Publisher pub;
 void processRecognation(const darknet_ros_msgs::BoundingBoxes::ConstPtr &var);

public:
 Objectdetection(ros::NodeHandle nh) : nh_(nh), priv_nh_("~") //,
        //sub(nh, "topic", 5) // constructor initialization form for the subscriber if needed
    {

        //pub = nh_.advertise< perception_msgs::RectArray >("rectInfo", 10);
        pub = nh_.advertise< perception_msgs::RectArray >("rectInfo", 10);
        sub = nh_.subscribe("/darknet_ros/bounding_boxes",  100, &recognitionspace::Objectdetection::processRecognation, this);

         
        

    }

    ~Objectdetection() {}
};
}



