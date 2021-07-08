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

  enum OpMode
  {
    OMD_PROFILE_POSITION_MODE = 1,
    OMD_PROFILE_VELOCITY_MODE = 3,
    OMD_HOMING_MODE = 6,
    OMD_CURRENT_MODE = -3
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
  };

}
#endif