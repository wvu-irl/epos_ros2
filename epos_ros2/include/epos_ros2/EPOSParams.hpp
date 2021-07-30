/**
 * @file EPOSParams.hpp
 * @author  Jared J Beard <jbeard6@mix.wvu.edu>
 * @version 1.0
 *
 * @section DESCRIPTION
 *
 * Contains related parameters and variables necessary for running EPOS
 * devies
 */

#ifndef EPOS_PARAMS_HPP
#define EPOS_PARAMS_HPP

#include <vector>
#include <string>

namespace epos2
{
  /// Defines operational modes for EPOS to command Maxon motors as described
  /// in the EPOS Command Library. Definition includes official names.
  enum OperationMode
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
  enum DeviceState
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

  /// 
  struct MaxonMotor
  {
    /// Motor index
    int index;
    /// Motor gear ratio
    double gear_ratio;
    /// Number of encoder counts per revolution
    int counts_per_rev;
    /// Radius of wheel (cm)
    double wheel_radius;
    /// Angular velocity upper limit (RPM)
    double ang_vel_limit;
    /// Angular acceleration upper limit (RPM^2)
    double acc_limit;
    /// Stall current of motors (A) can be used to set artificial stall limit
    double stall_current;
    /// Maximum allowable instantaneous current (A) can be user defined
    double instantaneous_current_limit;
    /// Maximum allowable continuous current (A) can be user defined
    double continuous_current_limit;
    /// Torque constant of motors (mNm/A)
    double kT;
    /// Current state of the corresponding EPOS device
    DeviceState state;
    /// Current operation mode for the motor
    OperationMode mode;
  };

  /// Parameters for EPOS controller, motor parameters (assumes single motor type), and logging parameters
  struct EPOSParams
  {
    // Motors
    /// Assume joints are defined semantically, this defines mapping to ids
    std::map<std::string, int> motor_ids;
    /// Defines mapping from motor id to index in list
    std::map<std::string, int> motor_inds;
    /// Parameters for each motor
    std::vector<MaxonMotor> motors;

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