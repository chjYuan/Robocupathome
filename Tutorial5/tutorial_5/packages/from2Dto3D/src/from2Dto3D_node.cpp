#include <ros/ros.h>
#include <from2Dto3D/from2Dto3D.h>
using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "from2Dto3D");
    ros::NodeHandle nh;
    from2Dto3DSpace::From2Dto3D node(nh);
    ros::spin();
    return 0;
}
