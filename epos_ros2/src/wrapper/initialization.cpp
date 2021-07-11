#include <epos_ros2/EPOSWrapper.hpp>

namespace epos2
{
    /////////////////////////////////////////////////////////////////////
    /***************************INITIALIZATION**************************/
    /////////////////////////////////////////////////////////////////////
    /**
       Opens device and subdevices

       @param Motors to open
       @return Success(0)/Failure(1) of commands
    */
    // int EPOSWrapper::open_devices()
    // {
    //    //Success of code
    //    int result = RETURN_FAILED;
    //    // Internal use of motor parameters
    //    char *device_name = new char[255];
    //    char *protocol_stack_name = new char[255];
    //    char *interface_name = new char[255];
    //    char *port_name = new char[255];

    //    strcpy(device_name, device_name_.c_str());
    //    strcpy(protocol_stack_name, protocol_stack_name_.c_str());
    //    strcpy(interface_name, interface_name_.c_str());
    //    strcpy(port_name, port_name_.c_str());

    //    //std::cout << "dev " << pDeviceName << "__Prot " << pProtocolStackName << "__Int " << pInterfaceName << "__Por" << pPortName << "__Code" << error_code_ << std::endl;

    //    // ROS_INFO("Open device...");
    //    //Opens device
    //    key_handle_ = VCS_OpenDevice(device_name, protocol_stack_name, interface_name, port_name, &error_code_);
    //    //checking device opened
    //    if (key_handle_ != 0 && error_code_ == 0)
    //    {
    //       unsigned int baud_rate = 0;
    //       unsigned int timeout = 0;

    //       if (VCS_GetProtocolStackSettings(key_handle_, &baud_rate, &timeout, &error_code_))
    //       {
    //          if (VCS_SetProtocolStackSettings(key_handle_, baud_rate_, timeout, &error_code_))
    //          {
    //             if (VCS_GetProtocolStackSettings(key_handle_, &baud_rate, &timeout, &error_code_))
    //             {
    //                if (baud_rate_ == (int)baud_rate)
    //                {
    //                   result = RETURN_SUCCESS;
    //                   // ROS_INFO("Device opened");
    //                }
    //             }
    //          }
    //       }
    //    }
    //    else
    //    {
    //       key_handle_ = 0;
    //    }
    //    // remove temporary device std::strings
    //    delete[] device_name;
    //    delete[] protocol_stack_name;
    //    delete[] interface_name;
    //    delete[] port_name;

    //    //Get initial device state here

    //    return result;
    // }

    // /**
    //     Closes device and subdevices

    //     @return Success(0)/Failure(1) of command
    //  */
    // int EPOSWrapper::close_devices()
    // {
    //    int result = RETURN_FAILED;
    //    error_code_ = 0;
    //    // ROS_INFO("Close device");

    //    if (VCS_CloseAllDevices(&error_code_) && error_code_ == 0)
    //    {
    //       result = RETURN_SUCCESS;
    //    }
    //    return result;
    // }

    /////////////////////////////////////////////////////////////////////
    /***************************CONSTRUCTORS****************************/
    /////////////////////////////////////////////////////////////////////

    ///
    ///
    ///
    EPOSWrapper::EPOSWrapper()
    {
        assertm(false, "No default initilializer: code is dependent on ROS Node Pointer!");
    }

    ///
    ///
    ///
    EPOSWrapper::EPOSWrapper(rclcpp::Node *_node_ptr, EPOSParams _epos_params) : node_ptr_(_node_ptr), epos_params_(_epos_params)
    {
        log_msg("Initializing EPOS Wrapper", LOG_INFO, LOG_PROGRESS);
        // Open devices
        //this->open_devices();
    }

    /**
        *   Destructor closes all devices :)
    */
    //EPOSWrapper::~EPOSWrapper()
    //{
    // if (!this->close_devices())
    // {
    // std::cerr << "Failed to close devices, try " << i << "."<< std::endl;
    /// NEED TO TRY TO CORRECT FAULTS HERE
    //if (!this->closeDevices())
    // print still failedds
    // }
    //}

}