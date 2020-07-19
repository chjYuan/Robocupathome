#include <ros/ros.h>
#include <tf_tiago/transform.h>
using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "transform");
    ros::NodeHandle nh;
    transformSpace::Transform node(nh);
    ros::spin();
    return 0;
}
