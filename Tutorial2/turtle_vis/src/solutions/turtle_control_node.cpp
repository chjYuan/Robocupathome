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
#include <math.h> 

/*********************************************************************
 * CUSTOM CLASS
 * ******************************************************************/
#include <turtle_vis/myClass/TurtleClass.h>
//#include <turtle_vis/DesiredPose.h>   


int main( int argc, char** argv )
{

    ros::init(argc, argv, "turtle_control",ros::init_options::AnonymousName);

    ROS_INFO_STREAM("**Publishing turtle control..");

    ros::NodeHandle n;
    ros::Rate r(60);



    //ADVERTISE THE SERVICE
    turtleSpace::TurtleClass turtleF;
    ros::ServiceServer service=n.advertiseService("TurtlePose",
                                                  &turtleSpace::TurtleClass::getDPose,
                                                  //#>>>>TODO: DEFINE THE CALLBACK FUNCTION,
                                                  &turtleF);
    //CALL SERVICE FROM TERMINAL//
    //    rosservice call /TurtlePose '{p: [0.5, 0.0, 3.0]}'
    //    rosservice call /TurtlePose "{p: {x: 1.5, y: 1.0, theta: 0.0}}"
    //DON'T FORGET TO SOURCE THE WORKSPACE BEFORE CALLING THE SERVICE

    //ADVERTIZE THE TOPIC command to move the robot
    ros::Publisher pub=n.advertise</*DESIRED_POSE_TOPIC*/ geometry_msgs::Twist>("/mobile_base_controller/cmd_vel",100);
    
    //get the current position from the sensors
    ros::Subscriber sub=n.subscribe("/mobile_base_controller/odom",100,
                                   &turtleSpace::TurtleClass::getPose,/*//#>>>>TODO: DEFINE THE CALLBACK FUNCTION FROM THE METHOD OF THE CLASS*/
                                   &turtleF);/*//#>>>>TODO: DEFINE THE INSTANCE OF THE CLASS*/
    ros::Rate loop_rate(3);
    
    ros::Time ti, tf;
    ti=ros::Time::now();

    //Proportional Gain
    Matrix2d Kp,Kt;
    //#>>>>TODO: SET GAINS
    double p_g=0.5;
    
    ROS_INFO_STREAM("p_g= "<<p_g);
     // K for Xd linear velocity
    Kp<<p_g,0,
            0,p_g; 
    
    ROS_INFO_STREAM("Kp= \n"<<Kp);

    Vector3d turtlePose;
    Vector2d error,turtleVel,T;
    float theta_c;
    float d= 0.3;
    double dt;

    //INITIALIZE THE TURTLE POSE
    turtlePose<<0,0,0;
    // initialize the liner velocity
    turtleVel<<0,0;

    //DESIRED POSE
    Vector3d turtlePose_desired_local;
    ////#>>>>TODO: INITIALIZE THE DESIRED POSE VARIABLE OF THE CLASS TURTLE 
    
    turtleF.turtlePose_desired_g=turtlePose;
    turtleF.turtlePose_g=turtlePose;
    //TURTLE_CLASS_MEMBER_VARIABLE=turtlePose;
    turtlePose_desired_local=turtlePose;


    //CREATE A DESIREDPOSE MSG VARIABLE
    // turtle_vis::DesiredPose msg; //#>>>>TODO:DEFINE THE MSG TYPE
    geometry_msgs::Twist msg;
    
    while(ros::ok())
    {
        tf=ros::Time::now();
 
        //  dt=tf.toSec()-ti.toSec();

        ////get the desired pose and the current pose of the reference point
        turtlePose_desired_local=turtleF.getLocalDesiredPose();
        turtlePose=turtleF.getLocalPose();
        turtlePose_desired_local[0]=turtlePose_desired_local[0]+d*cos(turtlePose_desired_local[2]);
        turtlePose_desired_local[1]=turtlePose_desired_local[1]+d*sin(turtlePose_desired_local[2]);
        turtlePose[0]=turtlePose[0]+d*cos(turtlePose[2]);
        turtlePose[1]=turtlePose[1]+d*sin(turtlePose[2]);
        //CONTROL
        ////#>>>>TODO:COMPUTE THE ERROR BETWEEN CURRENT POSE AND DESIRED
        error[0]= turtlePose_desired_local[0]-turtlePose[0];
        error[1]= turtlePose_desired_local[1]-turtlePose[1];
        // COMPUTE THE INCREMENTS
        turtleVel=Kp*error; //Xd
        theta_c=turtlePose[2];
        Kt<< cos(theta_c), (-0.3*sin(theta_c)),
                                sin(theta_c),(0.3*cos(theta_c));
        T=(Kt.inverse())*turtleVel; 
        //Publish Data
        msg.linear.x=T[0];
        msg.linear.y=0;
        msg.linear.z=0;     
        msg.angular.x=0;
        msg.angular.y=0;
        msg.angular.z=T[1];     
        // publish the current linear and angular velocity to the cmd to move the tiago
        pub.publish(msg);
        
        //SET THE HISTORY
        ti=tf;

        //ROS::SPIN IS IMPORTANT TO UPDATE ALL THE SERVICES AND TOPICS
        ros::spinOnce();

        r.sleep();
    }

    return 0;
}


