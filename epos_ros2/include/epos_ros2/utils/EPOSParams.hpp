/*
   Maxon Motor Controller EPOSWrapper Parameters
   EPOSParams.hpp
   Purpose: Link together various parameters for the EPOSWrapper

   @author Jared Beard
   @version 1.0 11/13/18
 */

#ifndef EPOS_PARAMS_HPP
#define EPOS_PARAMS_HPP

#include <vector>
#include <string>

namespace epos2
{
  /// Defines verbosity for custom logging
  enum loggingVerbosity
  {
    /// Turns off logging
    LOG_OFF = 0,
    /// Sets logging verbosity to ROS2 Debug
    LOG_DEBUG,
    /// Sets logging verbosity to ROS2 Info
    LOG_INFO,
    /// Sets logging verbosity to ROS2 Warn
    LOG_WARN,
    /// Sets logging verbosity to ROS2 Error
    LOG_ERROR,
    /// Sets logging verbosity to ROS2 Fatal
    LOG_FATAL
  };

  /// Defines group a given epos2::EPOSWrapper::log_msg call function belongs to.
  /// Helps with custom logging by given semantic meaning to logging calls
  /// (e.g., if we are monitoring the output of a motor we may use LOG_FEEDBACK to print motor position logs)
  enum loggingGroup
  {
    /// Includes all log output
    LOG_ALL = 0,
    /// Includes logging related to motor feedback (e.g., current, postion, etc)
    LOG_FEEDBACK,
    /// Includes logging related to motor input (e.g., velocity commands)
    LOG_INPUT,
    /// Includes logging related to current function call or other progress metrics
    LOG_PROGRESS,
    /// Includes logging related EPOS error messages
    LOG_ERROR,
    /// Print statements for unspecified groups
    LOG_OTHER
  };

  /// Defines operational modes for EPOS to command Maxon motors as described
  /// in the EPOS Command Library. Definition includes official names.
  enum OpMode
  {
    /// OMD_STEP_DIRECTION_MODE: 
    STEP_DIRECTION_MODE = -6,
    /// OMD_MASTER_ENCODER_MODE
    MASTER_ENCODER_MODE = -5,
    /// OMD_CURRENT_MODE
    CURRENT_MODE = -3,
    /// OMD_VELOCITY_MODE
    VELOCITY_MODE = -2,
    /// OMD_POSITION_MODE
    POSITION_MODE = -1,
    /// OMD_PROFILE_POSITION_MODE
    PROFILE_POSITION_MODE = 1,
    /// OMD_PROFILE_VELOCITY_MODE
    PROFILE_VELOCITY_MODE = 3,
    /// OMD_HOMING_MODE
    HOMING_MODE = 6,
    /// OMD_INTERPOLATED_POSITION_MODE
    INTERPOLATED_POSITION_MODE = 7
  };

  enum DevState
  {
    DISABLED = 0x0000,
    ENABLED = 0x0001,
    QUICKSTOP = 0x0002,
    FAULT = 0x0003
  };

  struct EPOSParams
  {
    // Motors
    std::map<std::string, int> motor_name_map;
    std::map<int, int> motor_ind_map;
    std::vector<int> motor_inds;
    double gear_ratio;
    int counts_per_rev;
    double wheel_radius;                // cm
    double ang_vel_limit;               // RPM
    double acc_limit;                   // RPM
    double stall_current;               // A
    double instantaneous_current_limit; // A
    double continuous_current_limit;    // A
    double kT = 31.5;                   // mNm/A

    // EPOS Modules
    int baud_rate = 1000000;
    std::string device_name = "EPOS4";
    std::string protocol_stack_name = "MAXON SERIAL V2";
    std::string interface_name = "USB";
    std::string port_name = "USB0";

    // Logging
    bool is_on;
    std::vector<int> groups;
  };

}
#endif