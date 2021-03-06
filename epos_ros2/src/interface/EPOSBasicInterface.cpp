#include <epos_ros2/EPOSWrapper.hpp>

namespace epos2
{
    /////////////////////////////////////////////////////////////////////
    /***************************OPERATION*******************************/
    /////////////////////////////////////////////////////////////////////

   // /**
   //     Clears fault for specified motor

   //     @param _id motor id
   //     @return Success(1)/Failure(0) of command
   //  */
   // int EPOSWrapper::handleFault(int _id)
   // {
   // 		BOOL is_fault = 0;
   // 		std::cout << "HF start" << std::endl;
   // 		if(VCS_GetFaultState(key_handle_, _id, &is_fault, &error_code_ ))
   // 		{
   // 				if(is_fault)
   // 				{
   // 						logError("VCS_GetFaultState");
   // 						if(VCS_ClearFault(key_handle_, _id, &error_code_) )
   // 						{
   // 								ROS_INFO("Fault Cleared Motor: %b", _id);
   // 								return RETURN_SUCCESS;
   // 						} else
   // 						{
   // 								logError("VCS_ClearFault");
   // 								ROS_INFO("Failed Fault Clear");
   // 								return RETURN_FAILED;
   // 						}
   // 				} else
   // 				{
   // 						ROS_DEBUG("No fault motor");
   // 				}
   // 		} else
   // 		{
   // 				ROS_WARN("Get Fault state failed");
   // 		}
   // }

   // /**
   //     Enables Specified motors

   //     @param _ids motor ids
   //     @return Success(1)/Failure(0) of command
   //  */
   // int EPOSWrapper::enableMotors(std::vector<int> _ids)
   // {
   // 		DevState state;
   // 		for (int i = 0; i < _ids.size(); ++i)
   // 		{
   // 				//std::cout << "Prepare Motors "<< std::endl;
   // 				if (getState(_ids[i], state))
   // 				{
   // 						//std::cout << "State " << state << " ;P" << std::endl;
   // 						if (state == FAULT)
   // 						{
   // 								std::cout << "FAULT" << std::endl;
   // 								handleFault(_ids[i]);
   // 								std::cout << "Clear Fault " << _ids[i] << std::endl;
   // 						}
   // 						if (state != ENABLED)
   // 						{
   // 								setState(_ids[i], ENABLED);
   // 						}
   // 				} else
   // 				{
   // 						ROS_WARN("Get state failed: motor %d", _ids[i]);
   // 				}
   // 		}
   // 		return RETURN_SUCCESS;
   // }

   // /************

   // NEED TO CHECK VELOCITY CONVERSION

   // *************/
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