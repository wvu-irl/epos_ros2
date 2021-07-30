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
    int EPOSWrapper::go_to_velocities(std::vector<std::string> _motors, std::vector<double> _velocities, bool _rpm = true)
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
    int EPOSWrapper::go_to_velocities(std::vector<std::string> _motors, double _velocity, bool _rpm = true)
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
    int EPOSWrapper::go_to_velocity(std::string _motor, double _velocity, bool _rpm = true)
    {
        RCLCPP_DEBUG(node_ptr_->get_logger(), "GO TO VELOCITY");

        if (params_.motors[params_.motor_ids[_motor]].mode != PROFILE_VELOCITY_MODE) 
            set_mode(_motor, PROFILE_VELOCITY_MODE);

        long velocity;
        if (_rpm)
        {
            velocity = _velocity;
        }
        else
        {
            //velocity = rpm_2_mps(_velocity);
        }

        // 				if (abs(_velocities[i]) > 0)
        // 				{
        // 						if (!VCS_MoveWithVelocity(key_handle_, _ids[i], _velocities[i],&error_code_))
        // 						{
        // 								logError("VCS_MoveWithVelocity");
        // 								return RETURN_FAILED;
        // 						} else {
        // 								ROS_INFO("Running");
        // 						}
        // 				} else
        // 				{
        // 						if (!VCS_HaltVelocityMovement(key_handle_, _ids[i],&error_code_))
        // 						{
        // 								return RETURN_FAILED;
        // 						}
        // 				}
        // 		return RETURN_SUCCESS;
    }

    ///
    ///
    ///
    int EPOSWrapper::get_velocities(std::vector<std::string> _motors, std::vector<double> &_velocities, bool _rpm = true)
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
    int EPOSWrapper::get_velocity(std::string _motor, double &_velocity, bool _rpm = true)
    {
    }

    ///
    ///
    ///
    int EPOSWrapper::go_to_positions(std::vector<std::string> _motors, std::vector<double> _positions, bool _count = true, bool _absolute = true, bool _immediate = true)
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
    int EPOSWrapper::go_to_positions(std::vector<std::string> _motors, double _position, bool _count = true, bool _absolute = true, bool _immediate = true)
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
    int EPOSWrapper::go_to_position(std::string _motor, double _position, bool _count = true, bool _absolute = true, bool _immediate = true)
    {
    }

    ///
    ///
    ///
    int EPOSWrapper::get_positions(std::vector<std::string> _motors, std::vector<double> &_positions, bool _count = true)
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
    int EPOSWrapper::get_position(std::string _motor, double &_position, bool _count = true)
    {
        // /**
        //     Gets position for a list of motors

        //     @param _ids motor ids
        // 		@param _positions angular position for each motor
        //     @return Success(1)/Failure(0) of command
        //  */
        // int EPOSWrapper::getPosition(std::vector<int> _ids, std::vector<int> &_positions) //128 cts/turn
        // {
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
        // }
    }

    ///
    ///
    ///
    int EPOSWrapper::go_to_torques(std::vector<std::string> _motors, std::vector<double> _torques)
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
    int EPOSWrapper::go_to_torques(std::vector<std::string> _motors, double _torque)
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
    int EPOSWrapper::go_to_torque(std::string _motor, double _torque)
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
    int EPOSWrapper::get_torques(std::vector<std::string> _motors, std::vector<double> &_torques)
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
    int EPOSWrapper::get_torque(std::string _motor, double &_torque)
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
    int EPOSWrapper::get_currents(std::vector<std::string> _motors, std::vector<double> &_currents)
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
    int EPOSWrapper::get_current(std::string _motor, double &_current)
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
    int EPOSWrapper::halt_velocity(std::string _motor)
    {
    }

    ///
    ///
    ///
    int EPOSWrapper::halt_position(std::string _motor)
    {
    }

}