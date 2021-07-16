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

  /// Defines device states for EPOS controllers
  enum DevState
  {
    /// Disable device 
    DISABLED = 0x0000,
    /// Enables device for normal operation
    ENABLED = 0x0001,
    /// Quick stop device
    QUICKSTOP = 0x0002,
    /// Device has faulted due to some condition
    FAULT = 0x0003
  };

  /// Parameters for EPOS controller, motor parameters (assumes single motor type), and logging parameters
  struct EPOSParams
  {
    // Motors
    /// Assume joints are defined semantically, this defines mapping to ids
    std::map<std::string, int> motor_name_map;
    /// Defines mapping from motor id to index in list
    std::map<int, int> motor_ind_map;
    /// List of motor indices
    std::vector<int> motor_inds;
    /// Motor gear ratio
    double gear_ratio;
    /// Number of encoder counts per revolution
    int counts_per_rev;
    /// Radius of wheel (cm) 
    double wheel_radius;                // cm
    /// Angular velocity upper limit (RPM)
    double ang_vel_limit;               // RPM
    /// Angular acceleration upper limit (RPM^2)
    double acc_limit;                   // RPM
    /// Stall current of motors (A) can be used to set artificial stall limit
    double stall_current;               // A
    /// Maximum allowable instantaneous current (A) can be user defined
    double instantaneous_current_limit; // A
    /// Maximum allowable continuous current (A) can be user defined
    double continuous_current_limit;    // A
    /// Torque constant of motors (mNm/A)
    double kT = 31.5;                   // mNm/A

    // EPOS Modules
    /// Baudrate for communication (Hz)
    int baud_rate = 1000000;
    /// Name of EPOS device (e.g. EPOS2 (not tested), EPOS4)
    std::string device_name = "EPOS4";
    /// Name of protocol stack for EPOS device
    std::string protocol_stack_name = "MAXON SERIAL V2";
    /// Name of interface being used (e.g. USB, RS232 (not tested))
    std::string interface_name = "USB";
    /// Port of interface
    std::string port_name = "USB0";

    // Logging
    /// If true sets logging to print
    bool is_on;
    /// Groups to log, see epos2::loggingGroup
    std::vector<int> groups;
  };

}
#endif