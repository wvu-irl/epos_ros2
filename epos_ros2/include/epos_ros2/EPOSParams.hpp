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
    std::vector<int> ids{1};
    int baudrate = 1000000;
    std::string device_name = "EPOS4";
    std::string protocol_stack_name = "MAXON SERIAL V2";
    std::string interface_name = "USB";
    std::string port_name = "USB0";
    int num_devices = 1;
  };

}
#endif