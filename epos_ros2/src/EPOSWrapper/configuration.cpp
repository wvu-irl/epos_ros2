#include <epos_ros2/EPOSWrapper.hpp>

namespace epos2
{
   /////////////////////////////////////////////////////////////////////
   /***************************CONFIGURATION***************************/
   /////////////////////////////////////////////////////////////////////

   ///
   ///
   ///
   int EPOSWrapper::set_modes(std::vector<std::string> _motors, std::vector<OperationMode> _modes)
   {
      log_msg("SET MODES", LOG_DEBUG, GROUP_PROGRESS);

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
   int EPOSWrapper::set_modes(std::vector<std::string> _motors, OperationMode _mode)
   {
      log_msg("SET MODES", LOG_DEBUG, GROUP_PROGRESS);

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
   int EPOSWrapper::set_mode(std::string _motor, OperationMode _mode)
   {
      log_msg("SET MODE", LOG_DEBUG, GROUP_PROGRESS);

      std::string msg;
      DWORD error_code;

      if (epos_params_.motors[epos_params_.motor_name_map[_motor]].mode == _mode)
      {
         msg = "Motor " + _motor + " set mode already " + std::to_string(_mode);
         log_msg(msg, LOG_DEBUG, GROUP_PROGRESS);
         return RETURN_SUCCESS;
      }
      else if (VCS_SetOperationMode(key_handle_, epos_params_.motor_name_map[_motor], _mode, &error_code))
      {
         msg = "Motor " + _motor + " set mode to " + get_mode_string(_mode);
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

   ///
   ///
   ///
   int EPOSWrapper::set_states(std::vector<std::string> _motors, std::vector<DeviceState> _states)
   {
      log_msg("SET STATES", LOG_DEBUG, GROUP_PROGRESS);

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
   int EPOSWrapper::set_states(std::vector<std::string> _motors, DeviceState _state)
   {
      log_msg("SET STATES", LOG_DEBUG, GROUP_PROGRESS);

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
   int EPOSWrapper::set_state(std::string _motor, DeviceState _state)
   {

      log_msg("SET STATE", LOG_DEBUG, GROUP_PROGRESS);

      std::string msg;
      DWORD error_code;

      if (VCS_SetState(key_handle_, epos_params_.motor_name_map[_motor], _state, &error_code))
      {
         msg = "Motor " + _motor + " state set to " + get_state_string(_state);
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
   int EPOSWrapper::get_states(std::vector<std::string> _motors, std::vector<DeviceState> &_states)
   {
      log_msg("GET STATES", LOG_DEBUG, GROUP_PROGRESS);

      _states.clear();
      DeviceState temp;

      for (auto &motor : _motors)
      {
         if (!this->get_state(motor,temp))
            return RETURN_FAILED;
         _states.push_back(temp);
      }
      return RETURN_SUCCESS;
   }

   ///
   ///
   ///
   int EPOSWrapper::get_state(std::string _motor, DeviceState &_state)
   {
      log_msg("GET STATE", LOG_DEBUG, GROUP_PROGRESS);

      std::string msg;
      DWORD error_code;
      WORD* state;

      if (VCS_GetState(key_handle_, epos_params_.motor_name_map[_motor], state, &error_code))
      {
         _state = device_state_map_[*state];
         msg = "Motor " + _motor + " state is " + get_state_string(_state);
         log_msg(msg, LOG_DEBUG, GROUP_PROGRESS);
         return RETURN_SUCCESS;
      }
      else
      {
         log_msg(this->get_error(error_code), LOG_WARN, GROUP_PROGRESS);
         return RETURN_FAILED;
      }
   }
}