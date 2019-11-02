#ifndef ROS_PARAMETER_HPP__
#define ROS_PARAMETER_HPP__

#include <ros/ros.h>
#include <XmlRpcException.h>

namespace ros {

// Load paramemeters from ROS parameter server
// throw ros::InvalidNameException if the key failed in validation
// throw ros::Exception for any XmlRpc exceptions
// throw ros::Exception is required is true, but parameter is not set
template <typename T>
void LoadParam(const ros::NodeHandle& nh, const std::string& name, T& value, bool required = false)
{
    try
    {
        if (nh.getParam(name, value))
            ROS_INFO_STREAM("Parameter is loaded: " << name << " = " << value);
        else if (required)
                throw ros::Exception("Failed loading parameter: " + name);
        else ROS_WARN_STREAM("Failed loading parameter, using defalut: " << name << " = " << value);
    }
    catch (const XmlRpc::XmlRpcException& ex)
    {
        throw ros::Exception(std::string("XmlRpcException: ") + ex.getMessage());
    }
}

// Load paramemeters from ROS parameter server
// throw ros::InvalidNameException if the key failed in validation
// throw ros::Exception for any XmlRpc exceptions
// throw ros::Exception is required is true, but parameter is not set
template <typename T>
void LoadParam(const ros::NodeHandle& nh, const std::string& name, std::vector<T>& value, bool required = false)
{
    try
    {
        bool loaded = nh.getParam(name, value);
        std::ostringstream oss;
        for (int i = 0; i < value.size(); i++)
        {
            oss << value[i] << " ";
        }

        if (loaded)
            ROS_INFO_STREAM("Parameter is loaded: " << name << " = " << oss.str());
        else if (required)
            throw ros::Exception("Failed loading parameter: " + name);
        else
            ROS_WARN_STREAM("Failed loading parameter, using defalut: " << name << " = " << oss.str());
    }
    catch (const XmlRpc::XmlRpcException& ex)
    {
        throw ros::Exception(std::string("XmlRpcException: ") + ex.getMessage());
    }
}

} // namespace ros 

#endif // #ifndef ROS_PARAMETER_HPP__
