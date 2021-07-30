/**
 * @file operation.cpp
 * @author  Jared J Beard <jbeard6@mix.wvu.edu>
 * @version 1.0
 *
 * @section DESCRIPTION
 *
 * Functions related to I/O for Maxon Motor devices
 * TODO: GET VELOCITY/POSITION/TORQUE UNITS AND CHECK CONVERSION
 * TODO: I believe vel is in RPM and Position in encoder counts, and Torque is A or mA
 *          as such, should make additional functions for m/s and m. Also should add a relative 
 *          position movement since MoveToPosition allows for this
 *          Additionally, there are some features fo position such as to wait until previous position met
 *          or just go right on ahead
 * TODO: Come back and throttle high frequency output
 * TODO: See if the movewith vel/toPos/currentmust functions convert the mode
 *          if so then those lines can be taken out and the code will be more efficient
 */
#include <epos_ros2/EPOSWrapper.hpp>

namespace epos2
{
    /////////////////////////////////////////////////////////////////////
    /***************************OPERATION*******************************/
    /////////////////////////////////////////////////////////////////////

    ///
    ///
    ///
    int EPOSWrapper::go_to_velocities(const std::vector<std::string> &_motors, const std::vector<double> &_velocities, bool _rpm = true)
    {
        RCLCPP_DEBUG(node_ptr_->get_logger(), "GO TO VELOCITIES");

        for (std::vector<std::string>::size_type i = 0; i < _motors.size(); ++i)
        {
            if (!this->go_to_velocity(_motors[i], _velocities[i], _rpm))
                return RETURN_FAILED;
        }
        return RETURN_SUCCESS;
    }

    ///
    ///
    ///
    int EPOSWrapper::go_to_velocities(const std::vector<std::string> &_motors, double _velocity, bool _rpm = true)
    {
        RCLCPP_DEBUG(node_ptr_->get_logger(), "GO TO VELOCITIES");

        for (auto &motor : _motors)
        {
            if (!this->go_to_velocity(motor, _velocity, _rpm))
                return RETURN_FAILED;
        }
        return RETURN_SUCCESS;
    }

    ///
    ///
    ///
    int EPOSWrapper::go_to_velocity(const std::string &_motor, double _velocity, bool _rpm = true)
    {
        RCLCPP_DEBUG(node_ptr_->get_logger(), "GO TO VELOCITY");

        std::string msg;
        DWORD error_code;

        //check motor is in correct mode
        if (params_.motors[params_.motor_inds[_motor]].mode != VELOCITY_MODE)
            set_mode(_motor, VELOCITY_MODE);

        // convert velocity if need be
        long velocity;
        if (_rpm)
        {
            velocity = _velocity;
        }
        else
        {
            velocity = rpm_2_mps(params_.motors[params_.motor_inds[_motor]], _velocity);
        }

        //command velocity or halt
        if (abs(_velocity) > 0)
        {
            if (VCS_MoveWithVelocity(key_handle_, params_.motor_ids[_motor], velocity, &error_code))
            {
                msg = "Motor " + _motor + " commanded to velocity of " + std::to_string(velocity) + " RPM";
                RCLCPP_DEBUG(node_ptr_->get_logger(), msg.c_str());
                return RETURN_SUCCESS;
            }
            else
            {
                RCLCPP_WARN(node_ptr_->get_logger(), this->get_error(error_code).c_str());
                return RETURN_FAILED;
            }
        }
        else
        {
            if (!halt_velocity(_motor))
                return RETURN_FAILED;
        }
        return RETURN_SUCCESS;
    }

    ///
    ///
    ///
    int EPOSWrapper::get_velocities(const std::vector<std::string> &_motors, std::vector<double> &_velocities, bool _rpm = true)
    {
        RCLCPP_DEBUG(node_ptr_->get_logger(), "GET VELOCITIES");

        _velocities.clear();
        double temp;

        for (auto &motor : _motors)
        {
            if (!this->get_velocity(motor, temp, _rpm))
                return RETURN_FAILED;
            _velocities.push_back(temp);
        }
        return RETURN_SUCCESS;
    }

    ///
    ///
    ///
    int EPOSWrapper::get_velocity(const std::string &_motor, double &_velocity, bool _rpm = true)
    {
        RCLCPP_DEBUG(node_ptr_->get_logger(), "GET VELOCITY");

        std::string msg;
        DWORD error_code;
        int *v;

        if (VCS_GetVelocityIs(key_handle_, params_.motor_ids[_motor], v, &error_code))
        {
            // convert velocity if need be
            if (!_rpm)
                _velocity = mps_2_rpm(params_.motors[params_.motor_inds[_motor]], *v);
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
    int EPOSWrapper::go_to_positions(const std::vector<std::string> &_motors, const std::vector<double> &_positions, bool _count = true, bool _absolute = true, bool _immediate = true)
    {
        RCLCPP_DEBUG(node_ptr_->get_logger(), "GO TO POSITIONS");

        for (std::vector<std::string>::size_type i = 0; i < _motors.size(); ++i)
        {
            if (!this->go_to_position(_motors[i], _positions[i], _count, _absolute, _immediate))
                return RETURN_FAILED;
        }
        return RETURN_SUCCESS;
    }

    ///
    ///
    ///
    int EPOSWrapper::go_to_positions(const std::vector<std::string> &_motors, double _position, bool _count = true, bool _absolute = true, bool _immediate = true)
    {
        RCLCPP_DEBUG(node_ptr_->get_logger(), "GO TO POSITIONS");

        for (auto &motor : _motors)
        {
            if (!this->go_to_position(motor, _position, _count, _absolute, _immediate))
                return RETURN_FAILED;
        }
        return RETURN_SUCCESS;
    }

    ///
    ///
    ///
    int EPOSWrapper::go_to_position(const std::string &_motor, double _position, bool _count = true, bool _absolute = true, bool _immediate = true)
    {
        RCLCPP_DEBUG(node_ptr_->get_logger(), "GO TO POSITION");

        std::string msg;
        DWORD error_code;

        //check motor is in correct mode
        if (params_.motors[params_.motor_inds[_motor]].mode != POSITION_MODE)
            set_mode(_motor, POSITION_MODE);

        // // convert velocity if need be
        // long position;
        // if (_count)
        // {
        //     position = _position;
        // }
        // else
        // {
        //     position = rpm_2_mps(params_.motors[params_.motor_inds[_motor]], _position);
        // }

        // //command position or halt
        // if (abs(_velocity) > 0)
        // {
        //     if (VCS_MoveWithVelocity(key_handle_, params_.motor_ids[_motor], velocity, &error_code))
        //     {
        //         msg = "Motor " + _motor + " commanded to velocity of " + std::to_string(velocity) + " RPM";
        //         RCLCPP_DEBUG(node_ptr_->get_logger(), msg.c_str());
        //         return RETURN_SUCCESS;
        //     }
        //     else
        //     {
        //         RCLCPP_WARN(node_ptr_->get_logger(), this->get_error(error_code).c_str());
        //         return RETURN_FAILED;
        //     }
        // }
        // else
        // {
        //     if (!halt_velocity(_motor))
        //         return RETURN_FAILED;
        // }
        // return RETURN_SUCCESS;
    }

    ///
    ///
    ///
    int EPOSWrapper::get_positions(const std::vector<std::string> &_motors, std::vector<double> &_positions, bool _count = true)
    {
        RCLCPP_DEBUG(node_ptr_->get_logger(), "GET POSITIONS");

        _positions.clear();
        double temp;

        for (auto &motor : _motors)
        {
            if (!this->get_position(motor, temp, _count))
                return RETURN_FAILED;
            _positions.push_back(temp);
        }
        return RETURN_SUCCESS;
    }

    ///
    ///
    ///
    int EPOSWrapper::get_position(const std::string &_motor, double &_position, bool _count = true)
    {

        // 		int pos = 0;
        // 		//ROS_WARN("---------------------------------------------------%d", pos);
        // 		//*pos = 1;
        // 		//ROS_WARN("---------------------------------------------------%d", *pos);
        // 		for (int i = 0; i < _ids.size(); ++i)
        // 		{
        // 				ROS_WARN("pos %d",  _ids[i]);
        // 				if (VCS_GetPositionIs(key_handle_, _ids[i], &pos, &error_code_))
        // 				{
        // 						ROS_WARN(" is %d", pos);
        // 						_positions.push_back(pos);
        // 						std::cout << " is " << pos << std::endl;
        // 				}
        // 				else
        // 				{
        // 						std::cout << " FAILED POSITION. " << std::endl;
        // 						return RETURN_FAILED;
        // 				}
        // 		}

        // 		return RETURN_SUCCESS;
    }

    ///
    ///
    ///
    int EPOSWrapper::go_to_torques(const std::vector<std::string> &_motors, const std::vector<double> &_torques)
    {
        RCLCPP_DEBUG(node_ptr_->get_logger(), "GO TO TORQUES");

        for (std::vector<std::string>::size_type i = 0; i < _motors.size(); ++i)
        {
            if (!this->go_to_torque(_motors[i], _torques[i]))
                return RETURN_FAILED;
        }
        return RETURN_SUCCESS;
    }

    ///
    ///
    ///
    int EPOSWrapper::go_to_torques(const std::vector<std::string> &_motors, double _torque)
    {
        RCLCPP_DEBUG(node_ptr_->get_logger(), "GO TO TORQUES");

        for (auto &motor : _motors)
        {
            if (!this->go_to_torque(motor, _torque))
                return RETURN_FAILED;
        }
        return RETURN_SUCCESS;
    }

    ///
    ///
    ///
    int EPOSWrapper::go_to_torque(const std::string &_motor, double _torque)
    {
        //    int EPOSWrapper::go_to_torque(std::vector<int> _ids, std::vector<long> _torques, double _gr)
        //    {
        //    		if (current_mode_ != OMD_CURRENT_MODE) setMode(_ids, OMD_CURRENT_MODE);

        //    		for (int i = 0; i < _ids.size(); ++i)
        //    		{
        //    				short current_amps = floor(_torques[i]/(kT_*_gr));
        //    				if (VCS_SetCurrentMust(key_handle_, _ids[i], current_amps,&error_code_))
        //    				{
        //    						ROS_INFO("Running Current");
        //    				} else {
        //    						logError("VCS_SetCurrentMust");
        //    						return RETURN_FAILED;
        //    				}

        //    		}
        //    		return RETURN_SUCCESS;
        //    }
    }

    ///
    ///
    ///
    int EPOSWrapper::get_torques(const std::vector<std::string> &_motors, std::vector<double> &_torques)
    {
        RCLCPP_DEBUG(node_ptr_->get_logger(), "GET TORQUES");

        _torques.clear();
        double temp;

        for (auto &motor : _motors)
        {
            if (!this->get_torque(motor, temp))
                return RETURN_FAILED;
            _torques.push_back(temp);
        }
        return RETURN_SUCCESS;
    }

    ///
    ///
    ///
    int EPOSWrapper::get_torque(const std::string &_motor, double &_torque)
    {
        // int EPOSWrapper::getCurrent(std::vector<int> _ids, std::vector<short> &_currents)
        // {
        //     	short current = 0;
        // 		//ROS_WARN("---------------------------------------------------%d", pos);
        // 		//*pos = 1;
        // 		//ROS_WARN("---------------------------------------------------%d", *pos);
        // 		for (int i = 0; i < _ids.size(); ++i)
        // 		{
        // 				ROS_WARN("current %d",  _ids[i]);
        // 				if (VCS_GetCurrentIs(key_handle_, _ids[i], &current, &error_code_))
        // 				{
        // 						ROS_WARN(" is %d", current);
        // 						_currents.push_back(current);
        // 						std::cout << " is " << current << std::endl;
        // 				}
        // 				else
        // 				{
        // 						std::cout << " FAILED CURRENT. " << std::endl;
        // 						return RETURN_FAILED;
        // 				}
        // 		}

        // 		return RETURN_SUCCESS;
        // }
    }

    ///
    ///
    ///
    int EPOSWrapper::get_currents(const std::vector<std::string> &_motors, std::vector<double> &_currents)
    {
        RCLCPP_DEBUG(node_ptr_->get_logger(), "GET CURRENTS");

        _currents.clear();
        double temp;

        for (auto &motor : _motors)
        {
            if (!this->get_current(motor, temp))
                return RETURN_FAILED;
            _currents.push_back(temp);
        }
        return RETURN_SUCCESS;
    }

    ///
    ///
    ///
    int EPOSWrapper::get_current(const std::string &_motor, double &_current)
    {
        // int EPOSWrapper::getCurrent(std::vector<int> _ids, std::vector<short> &_currents)
        // {
        //     	short current = 0;
        // 		//ROS_WARN("---------------------------------------------------%d", pos);
        // 		//*pos = 1;
        // 		//ROS_WARN("---------------------------------------------------%d", *pos);
        // 		for (int i = 0; i < _ids.size(); ++i)
        // 		{
        // 				ROS_WARN("current %d",  _ids[i]);
        // 				if (VCS_GetCurrentIs(key_handle_, _ids[i], &current, &error_code_))
        // 				{
        // 						ROS_WARN(" is %d", current);
        // 						_currents.push_back(current);
        // 						std::cout << " is " << current << std::endl;
        // 				}
        // 				else
        // 				{
        // 						std::cout << " FAILED CURRENT. " << std::endl;
        // 						return RETURN_FAILED;
        // 				}
        // 		}

        // 		return RETURN_SUCCESS;
        // }
    }

    ///
    ///
    ///
    int EPOSWrapper::halt_velocity(const std::string &_motor)
    {
        RCLCPP_DEBUG(node_ptr_->get_logger(), "HALT VELOCITY");

        std::string msg;
        DWORD error_code;

        if (VCS_HaltVelocityMovement(key_handle_, params_.motor_ids[_motor], &error_code))
        {
            msg = "Motor " + _motor + " halted";
            RCLCPP_DEBUG(node_ptr_->get_logger(), msg.c_str());
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
    int EPOSWrapper::halt_position(const std::string &_motor)
    {
        RCLCPP_DEBUG(node_ptr_->get_logger(), "HALT POSITION");

        std::string msg;
        DWORD error_code;

        if (VCS_HaltPositionMovement(key_handle_, params_.motor_ids[_motor], &error_code))
        {
            msg = "Motor " + _motor + " halted";
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