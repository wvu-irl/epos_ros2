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

        log_msg("Opening Devices", LOG_INFO, GROUP_PROGRESS);

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
        log_msg(msg, LOG_DEBUG, GROUP_PROGRESS);

        //Opens device
        key_handle_ = VCS_OpenDevice(device_name, protocol_stack_name, interface_name, port_name, &error_code);
        //msg = "Key handle: " + (int)key_handle_;
        //log_msg(msg, LOG_DEBUG, GROUP_PROGRESS);

        if (error_code != 0)
            log_msg(this->get_error(error_code), LOG_ERROR, GROUP_ERROR);

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
                    log_msg(msg, LOG_ERROR, GROUP_ALL);
                    log_msg(this->get_error(error_code), LOG_ERROR, GROUP_ERROR);
                }
                if (VCS_SetProtocolStackSettings(key_handle_, epos_params_.baud_rate, timeout, &error_code))
                {
                    if (error_code != 0)
                    {
                        msg = "Protocol Set: timeout: " + timeout;
                        log_msg(msg, LOG_ERROR, GROUP_ERROR);
                        log_msg(this->get_error(error_code), LOG_ERROR, GROUP_ERROR);
                    }
                    if (VCS_GetProtocolStackSettings(key_handle_, &baud_rate, &timeout, &error_code))
                    {
                        if (error_code != 0)
                        {
                            msg = "Protocol Get 3: timeout: " + timeout;
                            log_msg(msg, LOG_ERROR, GROUP_ERROR);
                            log_msg(this->get_error(error_code), LOG_ERROR, GROUP_ERROR);
                        }
                        if (epos_params_.baud_rate == (int)baud_rate)
                        {
                            result = RETURN_SUCCESS;
                            log_msg("Device Opened successfully", LOG_INFO, GROUP_PROGRESS);
                        }
                        else
                        {
                            log_msg("Baud rate did not match", LOG_WARN, GROUP_ERROR);
                        }
                    }
                    else
                    {
                        log_msg("VCS_GetProtocolStackSettings failed", LOG_ERROR, GROUP_ERROR);
                    }
                }
                else
                {
                    log_msg("VCS_GetProtocolStackSettings failed", LOG_ERROR, GROUP_ERROR);
                }
            }
            else
            {
                log_msg("VCS_GetProtocolStackSettings failed", LOG_ERROR, GROUP_ERROR);
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

        log_msg("Closing devices", LOG_INFO, GROUP_PROGRESS);

        if (VCS_CloseAllDevices(&error_code) && error_code == 0)
            result = RETURN_SUCCESS;

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
        log_msg("No default initilializer: code is dependent on ROS Node Pointer!", LOG_DEATH);
    }

    ///
    ///
    ///
    EPOSWrapper::EPOSWrapper(rclcpp::Node *_node_ptr, EPOSParams _epos_params) : node_ptr_(_node_ptr), epos_params_(_epos_params)
    {
        log_msg("Initializing EPOS Wrapper", LOG_WARN, GROUP_PROGRESS);
        // Open devices
        if (!this->open_devices())
            log_msg("DEVICES NOT OPENED", LOG_DEATH);
    }

    ///
    ///
    ///
    EPOSWrapper::~EPOSWrapper()
    {
        log_msg("Closing EPOS Devices/destructing EPOSWrapper", LOG_WARN, GROUP_PROGRESS);

        if (!this->close_devices())
            log_msg("FAILED TO CLOSE DEVICES", LOG_DEATH);
    }

}