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
        strcpy(device_name, epos_params_.device_name.c_str());
        strcpy(protocol_stack_name, epos_params_.protocol_stack_name.c_str());
        strcpy(interface_name, epos_params_.interface_name.c_str());
        strcpy(port_name, epos_params_.port_name.c_str());

        msg = "Device: " + epos_params_.device_name + " | Protocol Stack: " + epos_params_.protocol_stack_name +
              " | Interface: " + epos_params_.interface_name + " | Port: " + epos_params_.port_name;
        RCLCPP_WARN(node_ptr_->get_logger(), msg.c_str());

        //Opens device
        key_handle_ = VCS_OpenDevice(device_name, protocol_stack_name, interface_name, port_name, &error_code);
        //msg = "Key handle: " + (int)key_handle_;
        //log_msg(msg, LOG_DEBUG, GROUP_PROGRESS);

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
                if (VCS_SetProtocolStackSettings(key_handle_, epos_params_.baud_rate, timeout, &error_code))
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
                        if (epos_params_.baud_rate == (int)baud_rate)
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
    EPOSWrapper::EPOSWrapper(rclcpp::Node *_node_ptr, EPOSParams _epos_params) : node_ptr_(_node_ptr), epos_params_(_epos_params)
    {
        RCLCPP_WARN(node_ptr_->get_logger(), "Initializing EPOS Wrapper");
        // Open devices
        if (!this->open_devices())
        {
            RCLCPP_FATAL(node_ptr_->get_logger(), "Devices failed to open");
            assertm(true, "Killing EPOSWrapper");
        }
    }

    ///
    ///
    ///
    EPOSWrapper::~EPOSWrapper()
    {
        RCLCPP_WARN(node_ptr_->get_logger(), "Closing EPOS Devices/destructing EPOSWrapper");

        if (!this->close_devices())
            RCLCPP_FATAL(node_ptr_->get_logger(), "Devices failed to close");
    }

}