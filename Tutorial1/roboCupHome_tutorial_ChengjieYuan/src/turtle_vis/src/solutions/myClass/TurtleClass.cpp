#include<turtle_vis/myClass/TurtleClass.h>

namespace turtleSpace {

TurtleClass::TurtleClass()
{
    //#>>>>TODO: INITIALIZE MEMBER VARIABLES
    //Vector3d turtlePose_g,turtlePose_desired_g;
    
    turtlePose_g<<0 ,0, 0;
    turtlePose_desired_g<<0, 0, 0;
    
    count_mutex = PTHREAD_MUTEX_INITIALIZER;
}
TurtleClass::~TurtleClass()
{

}

void TurtleClass::getPose(const turtle_vis::DesiredPose::ConstPtr &msg)
{
    pthread_mutex_lock( &this->count_mutex );
    //#>>>>TODO: COPY THE MSG TO A LOCAL VARIABLE  ???
    turtlePose_g<< msg->x,msg->y,msg->theta;
    // here the msg is a constant pointer, is not an instance message,  should use the pointer pointing to the  internal value and do the copy
    //Form "const    class::XX::ConstPtr &XX "
    // turtlePose_g[1]=*msg.y; turtlePose_g[2]=*msg.theta; turtlePose_g=this->*msg; Wrong!!!!
    pthread_mutex_unlock( &this->count_mutex );

    //#>>>>TODO:PLOT THE OBTAINED DATA
    ROS_INFO("x:%f",turtlePose_g[0]);
    ROS_INFO("y:%f",turtlePose_g[1]);
    ROS_INFO("theta:%f",turtlePose_g[2]);
}

bool TurtleClass::getDPose(turtle_vis::send_desired_pose::Request &req, turtle_vis::send_desired_pose::Response &res)
{
    pthread_mutex_lock( &this->count_mutex );
    //#>>>>TODO:COPY THE REQUEST MSG TO A LOCAL VARIABLE 
    turtlePose_desired_g[0]=req.x;
    turtlePose_desired_g[1]=req.y;
    turtlePose_desired_g[2]=req.theta;
    // here the req is the instance message, we can use the member of it directly
    pthread_mutex_unlock( &this->count_mutex );

    //#>>>>TODO:PLOT THE OBTAINED DATA
    ROS_INFO("x:%f",turtlePose_desired_g[0]);
    ROS_INFO("y:%f",turtlePose_desired_g[1]);
    ROS_INFO("theta:%f",turtlePose_desired_g[2]);

    res.reply=1;

    return true;
}

Vector3d TurtleClass::getLocalPose()
{
    Vector3d local;
    pthread_mutex_lock( &this->count_mutex );
    local=this->turtlePose_g;
    pthread_mutex_unlock( &this->count_mutex );

    return local;
}

Vector3d TurtleClass::getLocalDesiredPose()
{
    Vector3d local;
    pthread_mutex_lock( &this->count_mutex );
    local=this->turtlePose_desired_g;
    pthread_mutex_unlock( &this->count_mutex );

    return local;
}
}