/**
 * @file operation.cpp
 * @author  Jared J Beard <jbeard6@mix.wvu.edu>
 * @version 1.0
 *
 * @section DESCRIPTION
 *
 * Functions related to I/O for Maxon Motor devices
 * TODO: GET VELOCITY/POSITION/TORQUE UNITS AND CHECK CONVERSION
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
    int EPOSWrapper::go_to_velocities(std::vector<std::string> _motors, std::vector<long> _velocities)
    {
    }

    ///
    ///
    ///
    int EPOSWrapper::go_to_velocities(std::vector<std::string> _motors, long _velocities)
    {
    }

    ///
    ///
    ///
    int EPOSWrapper::go_to_velocity(std::string _motor, long _velocity)
    {
    }

    ///
    ///
    ///
    int EPOSWrapper::get_velocities(std::vector<std::string> _motors, std::vector<long> &_velocities)
    {
    }

    ///
    ///
    ///
    int EPOSWrapper::get_velocity(std::string _motor, long &_velocity)
    {
    }

    ///
    ///
    ///
    int EPOSWrapper::go_to_positions(std::vector<std::string> _motors, std::vector<long> _positions)
    {
    }

    ///
    ///
    ///
    int EPOSWrapper::go_to_positions(std::vector<std::string> _motors, long _positions)
    {
    }

    ///
    ///
    ///
    int EPOSWrapper::go_to_position(std::string _motor, long _position)
    {
    }

    ///
    ///
    ///
    int EPOSWrapper::get_positions(std::vector<std::string> _motors, std::vector<long> &_positions)
    {
    }

    ///
    ///
    ///
    int EPOSWrapper::get_position(std::string _motor, long &_position)
    {
    }

    ///
    ///
    ///
    int EPOSWrapper::go_to_torques(std::vector<std::string> _motors, std::vector<long> _torques)
    {
    }

    ///
    ///
    ///
    int EPOSWrapper::go_to_torques(std::vector<std::string> _motors, long _torques)
    {
    }

    ///
    ///
    ///
    int EPOSWrapper::go_to_torque(std::string _motor, long _torque)
    {
    }

    ///
    ///
    ///
    int EPOSWrapper::get_torques(std::vector<std::string> _motors, std::vector<long> &_torques)
    {
    }

    ///
    ///
    ///
    int EPOSWrapper::get_torque(std::string _motor, long &_torque)
    {
    }

    ///
    ///
    ///
    // /**
    //     Sets velocity for a list of motors

    //     @param _ids motor ids
    // 		@param _velocities angular velocities for each motor. If only one velocity listed, it will be applied to all motor ids provided.
    //     @return Success(1)/Failure(0) of command
    //  */
    // int EPOSWrapper::goToVel(std::vector<int> _ids, std::vector<long> _velocities)
    // {
    // 		if (current_mode_ != OMD_PROFILE_VELOCITY_MODE) setMode(_ids, OMD_PROFILE_VELOCITY_MODE);

    // 		for (int i = 0; i < _ids.size(); ++i)
    // 		{
    // 				if (_velocities.size() < _ids.size() && i > 0) _velocities.push_back(_velocities[0]);

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
    // 		}
    // 		return RETURN_SUCCESS;
    // }

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

    // /**
    //     Gets current for a list of motors

    //     @param _ids motor ids
    // 		@param current current for each motor
    //     @return Success(1)/Failure(0) of command
    //  */
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