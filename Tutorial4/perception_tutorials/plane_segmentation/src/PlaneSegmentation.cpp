#include <plane_segmentation/PlaneSegmentation.h>


using namespace std;
using namespace cv;

//#>>>>TODO: FIX DEPENDENCIES IN CMakeLists.txt and package.xml (Make sure that everithing compiles in one shot.)

//#>>>>TODO: Separate this template in a class library (.h and .cpp files) and an node (.cpp file). The header files must be in a "include" folder in the package.

int main(int argc, char** argv)
{
    ros::init(argc, argv, "plane_segmentation");
    ros::NodeHandle nh;
    PlaneSegmentation node(nh);
    ros::spin();
    return 0;
}


