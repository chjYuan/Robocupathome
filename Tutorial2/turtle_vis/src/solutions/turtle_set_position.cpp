/*********************************************************************
* Compiler:         gcc 4.6.3
*
* Company:          Institute for Cognitive Systems
*                   Technical University of Munich
*
* Author:           Emmanuel Dean (dean@tum.de)
*                   Karinne Ramirez (karinne.ramirez@tum.de)
*
* Compatibility:    Ubuntu 12.04 64bit (ros hydro)
*
* Software Version: V0.1
*
* Created:          01.06.2015
*
* Comment:          turtle connection and visualization (Sensor and Signals)
*
********************************************************************/


/*********************************************************************
* STD INCLUDES
********************************************************************/
#include <iostream>
#include <fstream>
#include <pthread.h>


/*********************************************************************
* ROS INCLUDES
********************************************************************/
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

/*********************************************************************
* EIGEN INCLUDES
********************************************************************/
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <Eigen/Core>

/*********************************************************************
 * SEVICES AND MESSAGES
 * ******************************************************************/
//SET HEADERS FOR THE SERVICE AND THE MESSAGES OF THE TURTLE_VIS PACKAGE
#include <turtle_vis/DesiredPose.h>
#include <turtle_vis/send_desired_pose.h>

using namespace Eigen;


int main(int argc, char** argv)
{

    ros::init(argc, argv, "turtle_set_position",ros::init_options::AnonymousName);

    ROS_INFO_STREAM("**Client turtle desired position");

    ros::NodeHandle n;
    ros::Rate r(60);

    //INITIALIZE THE CLIENT
    ros::ServiceClient client=n.serviceClient</*TURTLE_SERVICE_TYPE*/ turtle_vis::send_desired_pose>("TurtlePose");/*//#>>>>TODO: DEFINE THE SERVICE TYPE*/

    ////#>>>>TODO: DEFINE A MSG VARIABLE FOR THE SERVICE MESSAGE
    turtle_vis::send_desired_pose msg;

    std::string myString;

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion qtf;

    while(ros::ok())
    {

        std::vector<double> vals;

        ROS_INFO_STREAM("Give me the desired position of the turtle: x_deisred,y_desired,theta");
        std::cin>>myString;

        ////#>>>>TODO:GET THE VALUES FROM THE TERMINAL AND SAVE THEM IN A LOCAL VARIABLE. YOU WILL GET X,Y AND THETA   ???is it necessary
        char *s_input = (char *)myString.c_str();
        const char * split = ",";
        char *p = strtok(s_input, split);
        float a; 
        while(p != NULL)
        { // char * -> int 
        sscanf(p, "%f", &a); 
        vals.push_back(a); 
        p=strtok(NULL, split); }
        //shujuleixing
         
        ////#>>>>TODO:CREATE THE MESSAGE WITH THE LOCAL VARIABLE
        msg.request.x=vals[0];
        msg.request.y=vals[1];
        msg.request.theta=vals[2];

        ////#>>>>TODO:COMPUTE THE POSITION AND ORIENTATION OF THE TF FOR THE DESIRED POSITION
        qtf.setRPY(0,0,msg.request.theta);////#>>>>TODO:USE THETA VARIABLE);
        transform.setOrigin(tf::Vector3(msg.request.x/*//#>>>>TODO:USE X VARIABLE*/, msg.request.y/*//#>>>>TODO:USE Y VARIABLE*/, 0));
        transform.setRotation(qtf);

        br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"/world","/turtle_desired"));


        if(client.call(msg))//#>>>>TODO:CALL THE CLIENT WITH msg)
        {
             ROS_INFO("x:%f",msg.request.x);
             ROS_INFO("y:%f",msg.request.y);
             ROS_INFO("theta:%f",msg.request.theta);

            //#>>>>TODO:PLOT THE MESSAGE
        }
        else
        {
            ROS_ERROR_STREAM("Failed to call the service 'TurtlePose'");
            return 1;
        }

    }



    return 0;
}
