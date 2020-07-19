/*********************************************************************************
    combined_resource_controller.cpp

    Example file for a controller plugin.

    The controllers are declared as a class inside a namespace stating the family of controllers.
    This class mus be a son of the controller_interface/controller_base class. The most important
    functions of the code below are:

    initRequest: This function contains the code to be executed at the moment the controller
                 manager calls the service to load the plugin. This function is executed only one
                 time before runing the controller's loop code. In this fucntion all variables
                 must be initialized, ros parameters read and all the objects constructed.

    update: This function contains the code to be executed every loop. The control law should
            be implemented here.

    stopping: Code to be executed one time when the controller is going to be unloaded. Here must
              be called all the destructors for the object and closed any port or comunication used
              on the controller.

**********************************************************************************/

#include <hardware_interface/internal/demangle_symbol.h>    // Hardware interface headers
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_state_interface.h>

#include <controller_interface/controller_base.h>           // Controller class base structure
#include <pluginlib/class_list_macros.h>                    // Header with tools to build plugins

#include <sensor_msgs/Imu.h>                                // Other libraries

using namespace hardware_interface;
using namespace std;

namespace controllers_tutorials{

class CombinedResourceController : public controller_interface::ControllerBase
{
public:

    bool initRequest(hardware_interface::RobotHW* robot_hw,
                     ros::NodeHandle&             root_nh,
                     ros::NodeHandle &            controller_nh,
                     ClaimedResources&       claimed_resources)
    {

        // Check if construction finished cleanly
        if (state_ != CONSTRUCTED)
        {
            ROS_ERROR("Cannot initialize this controller because it failed to be constructed");
            return false;
        }

        // Get a pointer to the joint position control interface
        PositionJointInterface* pos_iface = robot_hw->get<PositionJointInterface>();
        if (!pos_iface)
        {
            ROS_ERROR("This controller requires a hardware interface of type '%s'."
                      " Make sure this is registered in the hardware_interface::RobotHW class.",
                      getHardwareInterfaceType().c_str());
            return false;
        }

        // Get a pointer to the IMU sensor interface
        ImuSensorInterface* imu_iface = robot_hw->get<ImuSensorInterface>();
        if (!imu_iface)
        {
            ROS_ERROR("This controller requires a hardware interface of type '%s'."
                      " Make sure this is registered in the hardware_interface::RobotHW class.",
                      internal::demangledTypeName<ImuSensorInterface>().c_str());
            return false;
        }

        // Return which resources are claimed by this controller
        pos_iface->clearClaims();
        if (!init(pos_iface,
                  imu_iface,
                  root_nh,
                  controller_nh))
        {
            ROS_ERROR("Failed to initialize the controller");
            std::cerr  << "FAILED LOADING WALKING" << std::endl;
            return false;
        }
        //claimed_resources = pos_iface->getClaims();

        claimed_resources.push_back(InterfaceResources(internal::demangledTypeName<PositionJointInterface>(), pos_iface->getClaims()));
        pos_iface->clearClaims();

        // success
        state_ = INITIALIZED;
        return true;
    }

    bool init(PositionJointInterface*     pos_iface,
              ImuSensorInterface*         imu_iface,
              ros::NodeHandle&            /*root_nh*/,
              ros::NodeHandle&            controller_nh)
    {
        // Hardware interfaces
        if (!initJoints(pos_iface, controller_nh)     ||
                !initImuSensors(imu_iface, controller_nh) )
        {
            ROS_ERROR_STREAM("Failed to initialize controller '" << internal::demangledTypeName(*this) << "'");
            return false;
        }

        cont =0.0;
        return true;
    }

    bool initJoints(PositionJointInterface* pos_iface,
                    ros::NodeHandle&        controller_nh)
    {
        // Get joint names from the parameter server
        using namespace XmlRpc;
        XmlRpcValue joint_names;
        if (!controller_nh.getParam("joints", joint_names))
        {
            ROS_ERROR_STREAM("No joints given (namespace:" << controller_nh.getNamespace() << ").");
            return false;
        }
        if (joint_names.getType() != XmlRpcValue::TypeArray)
        {
            ROS_ERROR_STREAM("Malformed joint specification (namespace:" << controller_nh.getNamespace() << ").");
            return false;
        }

        // Populate temporary container of joint handles
        vector<hardware_interface::JointHandle> joints_tmp;
        for (int i = 0; i < joint_names.size(); ++i)
        {
            XmlRpcValue &name_value = joint_names[i];
            if (name_value.getType() != XmlRpcValue::TypeString)
            {
                ROS_ERROR_STREAM("Array of joint names should contain all strings (namespace:" << controller_nh.getNamespace() << ").");
                return false;
            }
            const string joint_name = static_cast<string>(name_value);

            // Get a joint handle
            try
            {
                joints_tmp.push_back(pos_iface->getHandle(joint_name));
                ROS_DEBUG_STREAM("Found joint '" << joint_name << "' in '" <<
                                 getHardwareInterfaceType() << "'");
            }
            catch (...)
            {
                ROS_ERROR_STREAM("Could not find joint '" << joint_name << "' in '" <<
                                 getHardwareInterfaceType() << "'");
                return false;
            }
        }

        // Member list of joint handles is updated only once all resources have been claimed
        joints_ = joints_tmp;

        return true;
    }

    bool initImuSensors(ImuSensorInterface* imu_iface,
                        ros::NodeHandle&    controller_nh)
    {
        // Base IMU
        string base_imu_name;
        if (!controller_nh.getParam("base_imu_sensor", base_imu_name))
        {
            ROS_ERROR_STREAM("No base_imu_sensor given (namespace:" << controller_nh.getNamespace() << ").");
            return false;
        }
        try
        {
            imu_sensor = imu_iface->getHandle(base_imu_name);

            if (!imu_sensor.getOrientation())
            {
                ROS_ERROR_STREAM("IMU sensor '" << base_imu_name << "' does not provide orientation readings");
                return false;
            }

            ROS_DEBUG_STREAM("Found IMU sensor '" << base_imu_name << "' in '" <<
                             internal::demangledTypeName(*imu_iface) << "'");
        }
        catch (...)
        {
            ROS_ERROR_STREAM("Could not find IMU sensor '" << base_imu_name << "' in '" <<
                             internal::demangledTypeName(*imu_iface) << "'");
            return false;
        }

        return true;
    }

    void update(const ros::Time& time, const ros::Duration& period)
    {
        // This time division is because of an error in gazebo.urdf.xacro
        // The simulation loops at a 1ms rate but the real robot loops at 5ms
        // This is one solution to this issue and MUST BE COMENTED when deploying
        // on the real robot.
        // Other posible solution is to fix it at the file /gazebo/gazebo.urdf.xacro
        // in the tiago_robot package.

        if( (time.nsec % 5000000) == 0)   // Coment this line when working on the real robot
        {

            cont+=0.01;
            double pos = 0.0;
            joints_[1].setCommand(0.3*sin(cont) - 1.3388);
            //ROS_WARN_STREAM("sin " << joints_[1].getPosition());

            //IMU
            sensor_msgs::Imu value;
            if (imu_sensor.getOrientation())
            {
                value.orientation.x = imu_sensor.getOrientation()[0];
                value.orientation.y = imu_sensor.getOrientation()[1];
                value.orientation.z = imu_sensor.getOrientation()[2];
                value.orientation.w = imu_sensor.getOrientation()[3];
            }
            //ROS_INFO_STREAM("IMU orientation x: "<<value.orientation.x<<" y: "<<value.orientation.y<<" z: "<<value.orientation.z<<" w: "<<value.orientation.w);
            ROS_WARN_STREAM("time: " << time.nsec );

        }
    }

    void stopping(const ros::Time& time)
    {}

    std::string getHardwareInterfaceType() const {return hardware_interface::internal::demangledTypeName<hardware_interface::PositionJointInterface>();}

private:
    // Definition of variable members for the controller class.
    double cont;

    // Define Hardware interface resources. This must match the stated on the config.yaml file.
    std::vector<hardware_interface::JointHandle> joints_;
    hardware_interface::ImuSensorHandle imu_sensor;
};

// This declares the class as a plugin library, the namespace and the class name must match the above code.
// The last argument of the function must remain as it is because it defines it as a plugin for the controller manager.
PLUGINLIB_DECLARE_CLASS(controllers_tutorials, CombinedResourceController, controllers_tutorials::CombinedResourceController, controller_interface::ControllerBase);

}
