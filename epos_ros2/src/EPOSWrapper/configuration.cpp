/**
 * @file configuration.cpp
 * @author  Jared J Beard <jbeard6@mix.wvu.edu>
 * @version 1.0
 *
 * @section DESCRIPTION
 *
 * Functions related to configuration of EPOS devices
 * TODO: (in reset_device) check what state reset goes to
 * TODO: Add get modes/state functions that just use internal variables so 
 *          users can decide whether or not its worth the risk to actually 
 *          probe device or have some trust, can affect comms if bandwidth 
 *          is too low
 * TODO: Come back and throttle high frequency output
 */
#include <epos_ros2/EPOSWrapper.hpp>

namespace epos2
{
    /////////////////////////////////////////////////////////////////////
    /***************************CONFIGURATION***************************/
    /////////////////////////////////////////////////////////////////////

    ///
    ///
    ///
    int EPOSWrapper::set_modes(const std::vector<std::string> &_motors, const std::vector<OperationMode> &_modes)
    {
        RCLCPP_DEBUG(node_ptr_->get_logger(), "SET MODES");

        for (std::vector<std::string>::size_type i = 0; i < _motors.size(); ++i)
        {
            if (!this->set_mode(_motors[i], _modes[i]))
                return RETURN_FAILED;
        }
        return RETURN_SUCCESS;
    }

    ///
    ///
    ///
    int EPOSWrapper::set_modes(const std::vector<std::string> &_motors, OperationMode _mode)
    {
        RCLCPP_DEBUG(node_ptr_->get_logger(), "SET MODES");

        for (auto &motor : _motors)
        {
            if (!this->set_mode(motor, _mode))
                return RETURN_FAILED;
        }
        return RETURN_SUCCESS;
    }

    ///
    ///
    ///
    int EPOSWrapper::set_mode(const std::string &_motor, OperationMode _mode)
    {
        RCLCPP_DEBUG(node_ptr_->get_logger(), "SET MODE");

        std::string msg;
        DWORD error_code;

        if (params_.motors[params_.motor_ids[_motor]].mode == _mode)
        {
            msg = "Motor " + _motor + " set mode already " + std::to_string(_mode);
            RCLCPP_DEBUG(node_ptr_->get_logger(), msg.c_str());
            return RETURN_SUCCESS;
        }
        else if (VCS_SetOperationMode(key_handle_, params_.motor_ids[_motor], _mode, &error_code))
        {
            msg = "Motor " + _motor + " set mode to " + get_mode_string(_mode);
            RCLCPP_DEBUG(node_ptr_->get_logger(), msg.c_str());
            params_.motors[params_.motor_ids[_motor]].mode = _mode;
            return RETURN_SUCCESS;
        }
        else
        {
            RCLCPP_WARN(node_ptr_->get_logger(), this->get_error(error_code).c_str());
            return RETURN_FAILED;
        }
    }

    ///
    ///
    ///
    int EPOSWrapper::reset_devices(const std::vector<std::string> &_motors)
    {
        RCLCPP_DEBUG(node_ptr_->get_logger(), "RESET DEVICES");

        for (auto &motor : _motors)
        {
            if (!this->reset_device(motor))
                return RETURN_FAILED;
        }
        return RETURN_SUCCESS;
    }

    ///
    int EPOSWrapper::reset_device(const std::string &_motor)
    {
        RCLCPP_DEBUG(node_ptr_->get_logger(), "RESET DEVICE");

        std::string msg;
        DWORD error_code;

        if (VCS_ResetDevice(key_handle_, params_.motor_ids[_motor], &error_code))
        {
            msg = "Motor " + _motor + " reset";
            RCLCPP_DEBUG(node_ptr_->get_logger(), msg.c_str());
            //TODO check what state of reset is and set that to internal variable
            return RETURN_SUCCESS;
        }
        else
        {
            RCLCPP_WARN(node_ptr_->get_logger(),this->get_error(error_code).c_str());
            return RETURN_FAILED;
        }
    }

    ///
    ///
    ///
    int EPOSWrapper::set_states(const std::vector<std::string> &_motors, const std::vector<DeviceState> &_states)
    {
        RCLCPP_DEBUG(node_ptr_->get_logger(), "SET STATES");

        for (std::vector<std::string>::size_type i = 0; i < _motors.size(); ++i)
        {
            if (!this->set_state(_motors[i], _states[i]))
                return RETURN_FAILED;
        }
        return RETURN_SUCCESS;
    }

    ///
    ///
    ///
    int EPOSWrapper::set_states(const std::vector<std::string> &_motors, DeviceState _state)
    {
        RCLCPP_DEBUG(node_ptr_->get_logger(), "SET STATES");

        for (auto &motor : _motors)
        {
            if (!this->set_state(motor, _state))
                return RETURN_FAILED;
        }
        return RETURN_SUCCESS;
    }

    /// Just sets state. Because this could have been changed onboard, have to assume
    /// that the current state value may be out of data. Manually checking would just be
    /// slower and require another function call to the system.
    int EPOSWrapper::set_state(const std::string &_motor, DeviceState _state)
    {

        RCLCPP_DEBUG(node_ptr_->get_logger(), "SET STATE");

        std::string msg;
        DWORD error_code;

        if (VCS_SetState(key_handle_, params_.motor_ids[_motor], _state, &error_code))
        {
            msg = "Motor " + _motor + " state set to " + get_state_string(_state);
            RCLCPP_DEBUG(node_ptr_->get_logger(), msg.c_str());
            params_.motors[params_.motor_ids[_motor]].state == _state;
            return RETURN_SUCCESS;
        }
        else
        {
            RCLCPP_WARN(node_ptr_->get_logger(),this->get_error(error_code).c_str());
            return RETURN_FAILED;
        }
    }

    ///
    ///
    ///
    int EPOSWrapper::get_states(const std::vector<std::string> &_motors, std::vector<DeviceState> &_states)
    {
        RCLCPP_DEBUG(node_ptr_->get_logger(), "GET STATES");

        _states.clear();
        DeviceState temp;

        for (auto &motor : _motors)
        {
            if (!this->get_state(motor, temp))
                return RETURN_FAILED;
            _states.push_back(temp);
        }
        return RETURN_SUCCESS;
    }

    ///
    ///
    ///
    int EPOSWrapper::get_state(const std::string &_motor, DeviceState &_state)
    {
        RCLCPP_DEBUG(node_ptr_->get_logger(), "GET STATE");

        std::string msg;
        DWORD error_code;
        WORD *state;

        if (VCS_GetState(key_handle_, params_.motor_ids[_motor], state, &error_code))
        {
            _state = device_state_map_[*state];
            msg = "Motor " + _motor + " state is " + get_state_string(_state);
            RCLCPP_DEBUG(node_ptr_->get_logger(), msg.c_str());
            return RETURN_SUCCESS;
        }
        else
        {
            RCLCPP_WARN(node_ptr_->get_logger(),this->get_error(error_code).c_str());
            return RETURN_FAILED;
        }
    }

        ///
    ///
    ///
    int EPOSWrapper::clear_faults(const std::vector<std::string> &_motors)
    {
        RCLCPP_DEBUG(node_ptr_->get_logger(), "CLEAR FAULTS");

        for (auto &motor : _motors)
        {
            if (!this->clear_fault(motor))
                return RETURN_FAILED;
        }
        return RETURN_SUCCESS;
    }

    ///
    ///
    ///
    int EPOSWrapper::clear_fault(const std::string &_motor)
    {
        RCLCPP_DEBUG(node_ptr_->get_logger(), "CLEAR FAULT");

        std::string msg;
        DWORD error_code;

        if (VCS_ClearFault(key_handle_, params_.motor_ids[_motor], &error_code))
        {
            msg = "Motor " + _motor + " fault cleared ";
            RCLCPP_DEBUG(node_ptr_->get_logger(), msg.c_str());
            return RETURN_SUCCESS;
        }
        else
        {
            RCLCPP_WARN(node_ptr_->get_logger(), this->get_error(error_code).c_str());
            return RETURN_FAILED;
        }
    }
}