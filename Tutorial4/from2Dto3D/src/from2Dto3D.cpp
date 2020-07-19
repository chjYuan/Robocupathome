
#include <from2Dto3D/from2Dto3D.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "from2Dto3D");
    ros::NodeHandle nh;
    ROS_INFO("Node from2Dto3D starting");

    From2Dto3D node(nh);

    //ros::Rate r(60);

    ros::spin();
    return 0;
}


