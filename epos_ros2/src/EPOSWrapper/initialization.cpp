/**
 * @file initialization.cpp
 * @author  Jared J Beard <jbeard6@mix.wvu.edu>
 * @version 1.0
 *
 * @section DESCRIPTION
 *
 * Functions related to startup and shutdown of devices/wrapper
 */
#include <epos_ros2/EPOSWrapper.hpp>

namespace epos2
{
    /////////////////////////////////////////////////////////////////////
    /***************************INITIALIZATION**************************/
    /////////////////////////////////////////////////////////////////////

    /// Uses VCS_OpenDevice(), VCS_GetProtocolStackSettings()
    ///
    ///
    int EPOSWrapper::open_devices()
    {
        // Variables
        std::string msg;  //For constructing log messages
        DWORD error_code; //For returning errors

        RCLCPP_WARN(node_ptr_->get_logger(), "Opening Devices");

        //Success of code
        int result = RETURN_FAILED;
        // Internal use of motor parameters
        char *device_name = new char[255];
        char *protocol_stack_name = new char[255];
        char *interface_name = new char[255];
        char *port_name = new char[255];
        // copy EPOS device parameters to local variables
        strcpy(device_name, params_.device_name.c_str());
        strcpy(protocol_stack_name, params_.protocol_stack_name.c_str());
        strcpy(interface_name, params_.interface_name.c_str());
        strcpy(port_name, params_.port_name.c_str());

        msg = "Device: " + params_.device_name + " | Protocol Stack: " + params_.protocol_stack_name +
              " | Interface: " + params_.interface_name + " | Port: " + params_.port_name;
        RCLCPP_WARN(node_ptr_->get_logger(), msg.c_str());

        //Opens device
        key_handle_ = VCS_OpenDevice(device_name, protocol_stack_name, interface_name, port_name, &error_code);

        // std::cout << "Keyhandle " << key_handle_ << " | " << error_code << std::endl;
        // msg = "Device: " + params_.device_name + " | Protocol Stack: " + params_.protocol_stack_name +
        //       " | Interface: " + params_.interface_name + " | Port: " + params_.port_name;
        // RCLCPP_WARN(node_ptr_->get_logger(), msg.c_str());


        if (error_code != 0)
            RCLCPP_ERROR(node_ptr_->get_logger(), this->get_error(error_code).c_str());

        //checking device opened
        if (key_handle_ != 0 && error_code == 0)
        {
            unsigned int baud_rate = 0;
            unsigned int timeout = 0;

            if (VCS_GetProtocolStackSettings(key_handle_, &baud_rate, &timeout, &error_code))
            {
                if (error_code != 0)
                {
                    msg = "Protocol Get 1: timeout: " + timeout;
                    RCLCPP_ERROR(node_ptr_->get_logger(), msg.c_str());
                    RCLCPP_ERROR(node_ptr_->get_logger(), this->get_error(error_code).c_str());
                }
                if (VCS_SetProtocolStackSettings(key_handle_, params_.baud_rate, timeout, &error_code))
                {
                    if (error_code != 0)
                    {
                        msg = "Protocol Set: timeout: " + timeout;
                        RCLCPP_ERROR(node_ptr_->get_logger(), msg.c_str());
                        RCLCPP_ERROR(node_ptr_->get_logger(), this->get_error(error_code).c_str());
                    }
                    if (VCS_GetProtocolStackSettings(key_handle_, &baud_rate, &timeout, &error_code))
                    {
                        if (error_code != 0)
                        {
                            msg = "Protocol Get 3: timeout: " + timeout;
                            RCLCPP_ERROR(node_ptr_->get_logger(), msg.c_str());
                            RCLCPP_ERROR(node_ptr_->get_logger(), this->get_error(error_code).c_str());
                        }
                        if (params_.baud_rate == (int)baud_rate)
                        {
                            result = RETURN_SUCCESS;
                            RCLCPP_WARN(node_ptr_->get_logger(), "Device Opened successfully");
                        }
                        else
                        {
                            RCLCPP_ERROR(node_ptr_->get_logger(), "Baud rate did not match");
                        }
                    }
                    else
                    {
                        RCLCPP_ERROR(node_ptr_->get_logger(), "VCS_GetProtocolStackSettings failed");
                    }
                }
                else
                {
                    RCLCPP_ERROR(node_ptr_->get_logger(), "VCS_GetProtocolStackSettings failed");
                }
            }
            else
            {
                RCLCPP_ERROR(node_ptr_->get_logger(), "VCS_GetProtocolStackSettings failed");
            }
        } 
        else
        {
            RCLCPP_ERROR(node_ptr_->get_logger(), "VCS_OpenDevice failed");
        }

        // remove temporary device std::strings
        delete[] device_name;
        delete[] protocol_stack_name;
        delete[] interface_name;
        delete[] port_name;

        return result;
    }

    /// Usess VCS_CloseAllDevices()
    ///
    ///
    int EPOSWrapper::close_devices()
    {
        int result = RETURN_FAILED;
        DWORD error_code = 0;

        RCLCPP_DEBUG(node_ptr_->get_logger(), "Closing devices");

        if (VCS_CloseAllDevices(&error_code) && error_code == 0)
        {
            RCLCPP_WARN(node_ptr_->get_logger(), "Devices closed");
            result = RETURN_SUCCESS;
        }
        else
        {
            RCLCPP_ERROR(node_ptr_->get_logger(), get_error(error_code).c_str());
        }
        return result;
    }

    /////////////////////////////////////////////////////////////////////
    /***************************CONSTRUCTORS****************************/
    /////////////////////////////////////////////////////////////////////

    ///
    ///
    ///
    EPOSWrapper::EPOSWrapper()
    {
        RCLCPP_FATAL(node_ptr_->get_logger(), "No default initilializer: code is dependent on ROS Node Pointer!");
        assertm(true, "Killing EPOSWrapper");
    }

    ///
    ///
    ///
    EPOSWrapper::EPOSWrapper(rclcpp::Node *_node_ptr, EPOSParams _params) : node_ptr_(_node_ptr), params_(_params)
    {
        this->clock_ = rclcpp::Clock();
        RCLCPP_WARN(node_ptr_->get_logger(), "Initializing EPOS Wrapper");
        // Open devices
        if (!this->open_devices())
        {
            RCLCPP_FATAL(node_ptr_->get_logger(), "Devices failed to open");
            assertm(true, "Killing EPOSWrapper");
        }

        // Enable motors here 
    }

    ///
    ///
    ///
    EPOSWrapper::~EPOSWrapper()
    {
        RCLCPP_WARN(node_ptr_->get_logger(), "Closing EPOS Devices/destructing EPOSWrapper");
        
        rclcpp::Time t = this->clock_.now();
        std::string msg;
        double dt = 0;

        bool sentinel = true;
        while(sentinel)
        {   
            
            sentinel = !halt_all_velocity();

            dt = (this->clock_.now() - t).seconds();
            if ( dt > (this->params_.motor_close_timeout * 1000) )
            {
                sentinel = false;
                RCLCPP_WARN(node_ptr_->get_logger(), "Too much time elapsed, device closed timeout");
            }
            msg = "Time elapsed " + std::to_string(dt);
            RCLCPP_WARN(node_ptr_->get_logger(), msg.c_str());
        }

        if (!this->close_devices())
        {
            RCLCPP_FATAL(node_ptr_->get_logger(), "Devices failed to close");
        }
    }

}