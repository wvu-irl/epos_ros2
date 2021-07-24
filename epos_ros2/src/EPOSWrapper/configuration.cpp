#include <epos_ros2/EPOSWrapper.hpp>

namespace epos2
{
   /////////////////////////////////////////////////////////////////////
   /***************************CONFIGURATION***************************/
   /////////////////////////////////////////////////////////////////////

   ///
   ///
   ///
   int EPOSWrapper::set_mode_motors(std::vector<std::string> _motors, std::vector<int> _modes)
   {
      log_msg("SET MODE MOTORS", LOG_DEBUG, GROUP_PROGRESS);

      for (std::vector<std::string>::size_type i = 0; i < _motors.size(); ++i)
      {
         if (!this->set_mode_motor(_motors[i], _modes[i]))
            return RETURN_FAILED;
      }
      return RETURN_SUCCESS;
   }

   ///
   ///
   ///
   int EPOSWrapper::set_mode_motors(std::vector<std::string> _motors, int _mode)
   {
      log_msg("SET MODE MOTORS", LOG_DEBUG, GROUP_PROGRESS);

      for (auto &motor : _motors)
      {
         if (!this->set_mode_motor(motor, _mode))
            return RETURN_FAILED;
      }
      return RETURN_SUCCESS;
   }

   ///
   ///
   ///
   int EPOSWrapper::set_mode_motor(std::string _motor, int _mode)
   {
      log_msg("SET MODE", LOG_DEBUG, GROUP_PROGRESS);

      std::string msg;
      DWORD error_code;

      if (VCS_SetOperationMode(key_handle_, epos_params_.motor_name_map[_motor], _mode, &error_code))
      {
         msg = "Motor " + _motor + " set to " + std::to_string(_mode);
         log_msg(msg, LOG_DEBUG, GROUP_PROGRESS);
         return RETURN_SUCCESS;
      }
      else
      {
         log_msg(this->get_error(error_code), LOG_WARN, GROUP_PROGRESS);
         return RETURN_FAILED;
      }
   }

   ///
   ///
   ///
   int EPOSWrapper::reset_devices(std::vector<std::string> _motors)
   {
      log_msg("RESET DEVICES", LOG_DEBUG, GROUP_PROGRESS);

      for (auto &motor : _motors)
      {
         if (!this->reset_device(motor))
            return RETURN_FAILED;
      }
      return RETURN_SUCCESS;
   }

   ///
   int EPOSWrapper::reset_device(std::string _motor)
   {
      log_msg("RESET DEVICE", LOG_DEBUG, GROUP_PROGRESS);

      std::string msg;
      DWORD error_code;

      if (VCS_ResetDevice(key_handle_, epos_params_.motor_name_map[_motor], &error_code))
      {
         msg = "Motor " + _motor + " reset";
         log_msg(msg, LOG_DEBUG, GROUP_PROGRESS);
         return RETURN_SUCCESS;
      }
      else
      {
         log_msg(this->get_error(error_code), LOG_WARN, GROUP_PROGRESS);
         return RETURN_FAILED;
      }
   }

   // /**
   //     Sets Device State in state machine

   //     @param _node_id node to have operation mode modified
   //     @param _state desried state of state machine
   //     @return Success(1)/Failure(0) of command
   //  */
   // int EPOSWrapper::setState(unsigned short _node_id, DevState _state)
   // {
   // 		if( current_state_ == _state || VCS_SetState(key_handle_,_node_id,_state,&error_code_))
   // 		{
   // 				return RETURN_SUCCESS;
   // 		} else
   // 		{
   // 				return RETURN_FAILED;
   // 		}
   // }

   // /**
   //     Gets Device State in state machine

   //     @param _node_id node to have operation mode modified
   //     @param _state state of state machine
   //     @return Success(1)/Failure(0) of command
   //  */
   // int EPOSWrapper::getState(unsigned short _node_id, DevState &_state)
   // {
   // 		short unsigned int state_value; // = getDevStateValue(state);
   // 		ROS_DEBUG("Retrieving State");
   // 		if( VCS_GetState(key_handle_,_node_id,&state_value,&error_code_))
   // 		{
   // 				ROS_DEBUG("State Retrieved");
   // 				_state = getDevState(state_value);
   // 				//std::cout << "Motor "<< nodeID << " is in state " << state << std::endl;
   // 				return RETURN_SUCCESS;

   // 		} else {
   // 				ROS_WARN("State Failed");
   // 				return RETURN_FAILED;
   // 		}
   // }
}