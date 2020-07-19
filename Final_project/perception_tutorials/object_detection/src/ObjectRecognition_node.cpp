#include "ros/ros.h"
#include "object_detection/ObjectRecognition.h"

using namespace recognitionspace;
int main(int argc, char** argv)
{
    ros::init(argc, argv, "ObjectRecognition");
    ros::NodeHandle nh;
    Objectdetection node(nh);
    ros::spin();
    return 0;
}

