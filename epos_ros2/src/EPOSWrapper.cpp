/*
   Maxon Motor Controller EPOSWrapper
   EPOSWrapper.cpp
   Purpose: Wrap EPOS commands and integrate with ROS for general purpose motor control

   @author Jared Beard
   @version 1.0 11/13/18
 */

#include <epos_ros2/EPOSWrapper.hpp>

namespace epos2
{

   /////////////////////////////////////////////////////////////////////
   /***************************INITIALIZATION**************************/
   /////////////////////////////////////////////////////////////////////
   /**
       Opens device and subdevices

       @param Motors to open
       @return Success(0)/Failure(1) of commands
    */
   // int EPOSWrapper::open_devices()
   // {
   //    //Success of code
   //    int result = RETURN_FAILED;
   //    // Internal use of motor parameters
   //    char *device_name = new char[255];
   //    char *protocol_stack_name = new char[255];
   //    char *interface_name = new char[255];
   //    char *port_name = new char[255];

   //    strcpy(device_name, device_name_.c_str());
   //    strcpy(protocol_stack_name, protocol_stack_name_.c_str());
   //    strcpy(interface_name, interface_name_.c_str());
   //    strcpy(port_name, port_name_.c_str());

   //    //std::cout << "dev " << pDeviceName << "__Prot " << pProtocolStackName << "__Int " << pInterfaceName << "__Por" << pPortName << "__Code" << error_code_ << std::endl;

   //    // ROS_INFO("Open device...");
   //    //Opens device
   //    key_handle_ = VCS_OpenDevice(device_name, protocol_stack_name, interface_name, port_name, &error_code_);
   //    //checking device opened
   //    if (key_handle_ != 0 && error_code_ == 0)
   //    {
   //       unsigned int baud_rate = 0;
   //       unsigned int timeout = 0;

   //       if (VCS_GetProtocolStackSettings(key_handle_, &baud_rate, &timeout, &error_code_))
   //       {
   //          if (VCS_SetProtocolStackSettings(key_handle_, baud_rate_, timeout, &error_code_))
   //          {
   //             if (VCS_GetProtocolStackSettings(key_handle_, &baud_rate, &timeout, &error_code_))
   //             {
   //                if (baud_rate_ == (int)baud_rate)
   //                {
   //                   result = RETURN_SUCCESS;
   //                   // ROS_INFO("Device opened");
   //                }
   //             }
   //          }
   //       }
   //    }
   //    else
   //    {
   //       key_handle_ = 0;
   //    }
   //    // remove temporary device std::strings
   //    delete[] device_name;
   //    delete[] protocol_stack_name;
   //    delete[] interface_name;
   //    delete[] port_name;

   //    //Get initial device state here

   //    return result;
   // }

   // /**
   //     Closes device and subdevices

   //     @return Success(0)/Failure(1) of command
   //  */
   // int EPOSWrapper::close_devices()
   // {
   //    int result = RETURN_FAILED;
   //    error_code_ = 0;
   //    // ROS_INFO("Close device");

   //    if (VCS_CloseAllDevices(&error_code_) && error_code_ == 0)
   //    {
   //       result = RETURN_SUCCESS;
   //    }
   //    return result;
   // }

   // /////////////////////////////////////////////////////////////////////
   // /***************************CONFIGURATION***************************/
   // /////////////////////////////////////////////////////////////////////
   // /**
   //     Sets mode for select devices

   //     @param _ids IDs for devices to set mode
   //     @param _mode operation mode to be set
   //     @return Success(0)/Failure(1) of command
   //  */
   // int EPOSWrapper::setMode(std::vector<int> _ids, OpMode _mode)
   // {
   // 		current_mode_ = _mode;
   // 		for (int i = 0; i < _ids.size(); ++i)
   // 		{
   // 				//std::cout << IDs.size() << std::endl;
   // 				if(VCS_SetOperationMode(key_handle_, _ids[i], _mode, &error_code_))
   // 				{
   // 						ROS_INFO("Operation mode set");
   // 				} else
   // 				{
   // 						logError("SetOperationMode");
   // 						return RETURN_FAILED;
   // 				}
   // 		}
   // 		return RETURN_SUCCESS;
   // }

   // /**
   //     ROS callback for setting operational mode

   //     @param msg ROS msg of custom type:
   //  */
   // /**void EPOSWrapper::setModeCallback(const )
   //    {
   //    for(int i = 0; i < msg.nodeID.size(); ++i)
   //    {
   //     if (!setMode(msg.nodeID[i],msg.mode, error_code_))
   //     {
   //       ROS_ERROR("FAILED TO SET MODE OF NODE %d", msg.nodeID[i]);
   //       break;
   //     }
   //    }
   //    }

   //    /**
   //     Resets device state machine

   //     @param _node_id node to have operation mode modified

   //     @return Success(0)/Failure(1) of command
   //  */
   // int EPOSWrapper::resetDevice(unsigned short _node_id)
   // {
   // 		int result = RETURN_FAILED;

   // 		if (VCS_ResetDevice(key_handle_, _node_id, &error_code_))
   // 		{
   // 				result = RETURN_SUCCESS;
   // 		}

   // 		return result;
   // }

   // /**
   //     Sets Device State in state machine

   //     @param _node_id node to have operation mode modified
   //     @param _state desried state of state machine
   //     @return Success(0)/Failure(1) of command
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
   //     @return Success(0)/Failure(1) of command
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

   // /////////////////////////////////////////////////////////////////////
   // /***************************OPERATION*******************************/
   // /////////////////////////////////////////////////////////////////////

   // /**
   //     Get short unsigned integer value for a state

   //     @param _state state of interest
   //     @return state integer value
   //  */
   // short unsigned int EPOSWrapper::getDevStateValue(DevState _state){
   // 		short unsigned int disabled = 0x0000;
   // 		short unsigned int enabled = 0x0001;
   // 		short unsigned int quickstop = 0x0002;
   // 		short unsigned int fault = 0x0003;

   // 		if (_state == DISABLED) {
   // 				return disabled;
   // 		} else if (_state == ENABLED) {
   // 				return enabled;
   // 		} else if (_state == QUICKSTOP) {
   // 				return quickstop;
   // 		} else if (_state == FAULT) {
   // 				return fault;
   // 		} else {
   // 				std::cout << "Invalid DevState" << std::endl;
   // 				return 8;
   // 		}
   // }

   // /**
   //     Get integer value for a mode

   //     @param _mode mode of interest
   //     @return mode integer value
   //  */
   // int EPOSWrapper::getModeValue(OpMode _mode){
   // 		int position = 1;
   // 		int velocity = 3;
   // 		int homing = 6;
   // 		int current = -3;

   // 		if (_mode == position) {
   // 				return position;
   // 		} else if (_mode == velocity) {
   // 				return velocity;
   // 		} else if (_mode == homing) {
   // 				return homing;
   // 		} else if (_mode == current) {
   // 				return current;
   // 		} else {
   // 				std::cout << "Invalid OpMode" << std::endl;
   // 				return 8;
   // 		}
   // }

   // /**
   //     Get state for a state integer

   //     @param _state state of interest
   //     @return state enum value
   //  */
   // enum EPOSWrapper::DevState EPOSWrapper::getDevState(short unsigned int _state)
   // {
   // 		short unsigned int disabled = 0x0000;
   // 		short unsigned int enabled = 0x0001;
   // 		short unsigned int quickstop = 0x0002;
   // 		short unsigned int fault = 0x0003;

   // 		if (_state == disabled) {
   // 				return DISABLED;
   // 		} else if (_state == enabled) {
   // 				return ENABLED;
   // 		} else if (_state == quickstop) {
   // 				return QUICKSTOP;
   // 		} else if (_state == fault) {
   // 				return FAULT;
   // 		} else {
   // 				ROS_WARN("Invalid DevStateValues");
   // 				return FAULT;
   // 		}
   // }

   // /**
   //     Clears fault for specified motor

   //     @param _id motor id
   //     @return Success(0)/Failure(1) of command
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
   //     @return Success(0)/Failure(1) of command
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
   //     @return Success(0)/Failure(1) of command
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
   //     @return Success(0)/Failure(1) of command
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
   //     @return Success(0)/Failure(1) of command
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

   // int EPOSWrapper::goToTorque(std::vector<int> _ids, std::vector<long> _torques, double _gr)
   // {
   // 		if (current_mode_ != OMD_CURRENT_MODE) setMode(_ids, OMD_CURRENT_MODE);

   // 		for (int i = 0; i < _ids.size(); ++i)
   // 		{
   // 				short current_amps = floor(_torques[i]/(kT_*_gr));
   // 				if (VCS_SetCurrentMust(key_handle_, _ids[i], current_amps,&error_code_))
   // 				{
   // 						ROS_INFO("Running Current");
   // 				} else {
   // 						logError("VCS_SetCurrentMust");
   // 						return RETURN_FAILED;
   // 				}

   // 		}
   // 		return RETURN_SUCCESS;
   // }

   /////////////////////////////////////////////////////////////////////
   /***************************CONSTRUCTORS****************************/
   /////////////////////////////////////////////////////////////////////
   /**
    Default Constructor
 */
   EPOSWrapper::EPOSWrapper()
   {
      assertm(false, "No default initilializer: code is dependent on ROS Node Pointer!");
   }

   /**
    Constructor for specific motor ids and baudrate

    @param ids id number of motors to be used
    @param br baudrate for communications
 */
   EPOSWrapper::EPOSWrapper(EPOSParams _epos_params) : epos_params_(_epos_params) //, ROSNodeParams _node_params) epos_params_(_epos_params), node_params_(_node_params)
   {

      //this->opendevices;
   }

   /**
       Destructor closes all devices
    */
   EPOSWrapper::~EPOSWrapper()
   {
      if (!this->close_devices())
      {
         // std::cerr << "Failed to close devices, try " << i << "."<< std::endl;
         /// NEED TO TRY TO CORRECT FAULTS HERE
         //if (!this->closeDevices())
         // print still failedds
      }
   }

   // /////////////////////////////////////////////////////////////////////
   // /***************************PRINT/DEBUGGING*************************/
   // /////////////////////////////////////////////////////////////////////
   // /**
   //     Displays Error info for an executed function

   //     @param error_code_Value Error code number
   //     @return Success(0)/Failure(1) of command
   //  */
   // int EPOSWrapper::getError(unsigned short _error_code_value)
   // {
   // 		int result = RETURN_FAILED;
   // 		if(VCS_GetErrorInfo(_error_code_value, error_code_char_, MMC_MAX_LOG_MSG_SIZE))
   // 		{
   // 				ROS_ERROR("ERROR %u: %u\n", _error_code_value, *error_code_char_);
   // 				result = RETURN_SUCCESS;
   // 		}

   // 		return result;
   // }

   // void EPOSWrapper::logError(std::string _function_name)
   // {
   // 		std::cerr << "EPOS COMMAND: " << _function_name << " failed (error_code_=0x" << std::hex << error_code_ << ")"<< std::endl;
   // }

   // int EPOSWrapper::checkNodeID(int _id)
   // {
   // 		int result = RETURN_FAILED;

   // 		for (int i = 0; i < node_id_list_.size(); ++i)
   // 		{
   // 				if (_id == node_id_list_[i]) {
   // 						result == RETURN_SUCCESS;
   // 				}
   // 		}
   // 		return result;
   // }

   std::string EPOSWrapper::getErrorCode(DWORD _error_code)
   {
      switch (_error_code)
      {
      case 0x00000000:
         return "[" + std::to_string(_error_code) + "] No error: communication was successful";
      case 0x10000001:
         return "[" + std::to_string(_error_code) + "] Internal error: ¯\\_(ツ)_/¯";
      case 0x10000002:
         return "[" + std::to_string(_error_code) + "] Null pointer: Null pointer passed to function";
      case 0x10000003:
         return "[" + std::to_string(_error_code) + "] Handle not valid: Handle passed to function is not valid";
      case 0x10000004:
         return "[" + std::to_string(_error_code) + "] Bad virtual device name: Virtual device name is not valid";
      case 0x10000005:
         return "[" + std::to_string(_error_code) + "] Bad device name: Device name is not valid";
      case 0x10000006:
         return "[" + std::to_string(_error_code) + "] Bad protocal stack name: Protocol stack name is not valid";
      case 0x10000007:
         return "[" + std::to_string(_error_code) + "] Bad interface name: Interface name is not valid";
      case 0x10000008:
         return "[" + std::to_string(_error_code) + "] Bad port name: Port name is not valid";
      case 0x10000009:
         return "[" + std::to_string(_error_code) + "] Library not loaded: Could not load external library";
      case 0x1000000A:
         return "[" + std::to_string(_error_code) + "] Command failed: Error while executing command";
      case 0x1000000B:
         return "[" + std::to_string(_error_code) + "] Timeout: Timeout occurred during execution";
      case 0x1000000C:
         return "[" + std::to_string(_error_code) + "] Bad parameter: Bad parameter passed to function";
      case 0x1000000D:
         return "[" + std::to_string(_error_code) + "] Command aborted by user: ¯\\_(ツ)_/¯";
      case 0x1000000E:
         return "[" + std::to_string(_error_code) + "] Buffer too small: ¯\\_(ツ)_/¯";
      case 0x1000000F:
         return "[" + std::to_string(_error_code) + "] No communication found: No communication settings found";
      case 0x10000010:
         return "[" + std::to_string(_error_code) + "] Function not supported: ¯\\_(ツ)_/¯";
      case 0x100000011:
         return "[" + std::to_string(_error_code) + "] Parameter already used: ¯\\_(ツ)_/¯";
      case 0x10000013:
         return "[" + std::to_string(_error_code) + "] Bad device handle: ¯\\_(ツ)_/¯";
      case 0x10000014:
         return "[" + std::to_string(_error_code) + "] Bad protocol stack: ¯\\_(ツ)_/¯";
      case 0x10000015:
         return "[" + std::to_string(_error_code) + "] Bad interface handle: ¯\\_(ツ)_/¯";
      case 0x10000016:
         return "[" + std::to_string(_error_code) + "] Bad port handle: ¯\\_(ツ)_/¯";
      case 0x10000017:
         return "[" + std::to_string(_error_code) + "] Address parameters are not correct: ¯\\_(ツ)_/¯";
      case 0x10000020:
         return "[" + std::to_string(_error_code) + "] Bad device state: ¯\\_(ツ)_/¯";
      case 0x10000021:
         return "[" + std::to_string(_error_code) + "] Bad file content: ¯\\_(ツ)_/¯";
      case 0x10000022:
         return "[" + std::to_string(_error_code) + "] Path does not exist: System cannot find specified path";
      case 0x10000024:
         return "[" + std::to_string(_error_code) + "] Cross thread error: (.NET only) Open device and close device called from different threads";
      case 0x10000026:
         return "[" + std::to_string(_error_code) + "] Gateway support error: Gateway is not supported";
      case 0x10000027:
         return "[" + std::to_string(_error_code) + "] Serial number update error: Serial number update failed";
      case 0x10000028:
         return "[" + std::to_string(_error_code) + "] Communication interface error: Communication interface is not supported";
      case 0x10000029:
         return "[" + std::to_string(_error_code) + "] Firmware support error: Firmware version does not support functionality";
      case 0x1000002A:
         return "[" + std::to_string(_error_code) + "] Firmware file hardware error: Firmware file does not match hardware version";
      case 0x1000002B:
         return "[" + std::to_string(_error_code) + "] Firmware file error: Firmware file does not match or is corrupt";
      case 0x20000001:
         return "[" + std::to_string(_error_code) + "] Opening interface error: Error while opening interface";
      case 0x20000002:
         return "[" + std::to_string(_error_code) + "] Closing interface error: Error while closing interface";
      case 0x20000003:
         return "[" + std::to_string(_error_code) + "] Interface is not open: ¯\\_(ツ)_/¯";
      case 0x20000004:
         return "[" + std::to_string(_error_code) + "] Opening port error: Error while opening port";
      case 0x20000005:
         return "[" + std::to_string(_error_code) + "] Closing port error: Error while closing port";
      case 0x20000006:
         return "[" + std::to_string(_error_code) + "] Port is not open: ¯\\_(ツ)_/¯";
      case 0x20000007:
         return "[" + std::to_string(_error_code) + "] Resetting port error: Error while resetting port";
      case 0x20000008:
         return "[" + std::to_string(_error_code) + "] Configuring port settings error: Error while configuring port settings";
      case 0x20000009:
         return "[" + std::to_string(_error_code) + "] Configuring port mode error: Error while configuring port mode";
      case 0x21000001:
         return "[" + std::to_string(_error_code) + "] RS232 write data error: Error while writing RS232 data";
      case 0x21000002:
         return "[" + std::to_string(_error_code) + "] RS232 read data error: Error while reading RS232 data";
      case 0x22000001:
         return "[" + std::to_string(_error_code) + "] CAN receive frame error: Error while receiving CAN frame";
      case 0x22000002:
         return "[" + std::to_string(_error_code) + "] CAN transmit frame error: Error while transmitting CAN frame";
      case 0x23000001:
         return "[" + std::to_string(_error_code) + "] USB write data error: Error while writing USB data";
      case 0x23000002:
         return "[" + std::to_string(_error_code) + "] USB read data error: Error while writing USB data";
      case 0x24000001:
         return "[" + std::to_string(_error_code) + "] HID write data error: Error while writing USB data to HID device";
      case 0x24000002:
         return "[" + std::to_string(_error_code) + "] HID read data error: Error while reading USB data from HID device";
      case 0x31000001:
         return "[" + std::to_string(_error_code) + "] Negative acknowledge received: ¯\\_(ツ)_/¯";
      case 0x31000002:
         return "[" + std::to_string(_error_code) + "] Bad CRC received: Bad checksum received";
      case 0x31000003:
         return "[" + std::to_string(_error_code) + "] Bad data received: Bad data size received";
      case 0x32000001:
         return "[" + std::to_string(_error_code) + "] SDO response not received: CAN frame of SDO protocol not received";
      case 0x32000002:
         return "[" + std::to_string(_error_code) + "] Requested CAN frame not received: ¯\\_(ツ)_/¯";
      case 0x32000003:
         return "[" + std::to_string(_error_code) + "] CAN frame not received: ¯\\_(ツ)_/¯";
      case 0x34000001:
         return "[" + std::to_string(_error_code) + "] Stuffing error: Failure while stuffing data";
      case 0x34000002:
         return "[" + std::to_string(_error_code) + "] Destuffing error: Failuer while destuffing data";
      case 0x34000003:
         return "[" + std::to_string(_error_code) + "] Bad CRC received: ¯\\_(ツ)_/¯";
      case 0x34000004:
         return "[" + std::to_string(_error_code) + "] Bad data size received: ¯\\_(ツ)_/¯";
      case 0x34000005:
         return "[" + std::to_string(_error_code) + "] Bad data size written: ¯\\_(ツ)_/¯";
      case 0x34000006:
         return "[" + std::to_string(_error_code) + "] Serial data frame not writter: Failuer occurred while writing data";
      case 0x34000007:
         return "[" + std::to_string(_error_code) + "] Serial data frame not received: Failure occurred while reading data";
      case 0x50300000:
         return "[" + std::to_string(_error_code) + "] Toggle error: Toggle bit not alternated";
      case 0x50400000:
         return "[" + std::to_string(_error_code) + "] SDO timeout: SDO protocol timed out";
      case 0x50400001:
         return "[" + std::to_string(_error_code) + "] Client/server specific error: Client/server command specifier not valid or unknown";
      case 0x50400002:
         return "[" + std::to_string(_error_code) + "] Invalid block size: (block mode only)";
      case 0x50400003:
         return "[" + std::to_string(_error_code) + "] Invalid sequence: Invalid sequence number (block mode only)";
      case 0x50400004:
         return "[" + std::to_string(_error_code) + "] CRC error: (block mode only)";
      case 0x5040005:
         return "[" + std::to_string(_error_code) + "] Out of memory error: ¯\\_(ツ)_/¯";
      case 0x51000001:
         return "[" + std::to_string(_error_code) + "] Bad data size received: Object data size does not correspond to requested data size";
      case 0x51000007:
         return "[" + std::to_string(_error_code) + "] Sensor configuration not supported: Sensor configuration cannot be writted to controller";
      case 0x51000008:
         return "[" + std::to_string(_error_code) + "] Sensor configuration unknown: Sensor configuration read from controller is not supported by library";
      case 0x51000009:
         return "[" + std::to_string(_error_code) + "] Configuration not supported: ¯\\_(ツ)_/¯";
      case 0x5100000A:
         return "[" + std::to_string(_error_code) + "] Digital input mask not supported: ¯\\_(ツ)_/¯";
      case 0x60100000:
         return "[" + std::to_string(_error_code) + "] Access error: Unsupported access to an object (e.g., write command to a read-only object";
      case 0x60100001:
         return "[" + std::to_string(_error_code) + "] Write only: Read command to a write only object";
      case 0x60100002:
         return "[" + std::to_string(_error_code) + "] Read only: Write command to a read only object";
      case 0x60200000:
         return "[" + std::to_string(_error_code) + "] Object does not exist: Last read/write command had an invalid object index or subindex";
      case 0x60400041:
         return "[" + std::to_string(_error_code) + "] PDO mapping error: Object cannot be mapped to PDO";
      case 0x60400042:
         return "[" + std::to_string(_error_code) + "] PDO length error: Number and length of objects to be mapped would exceed PDO length";
      case 0x60400043:
         return "[" + std::to_string(_error_code) + "] General parameter error: General parameter incompatibility";
      case 0x60400047:
         return "[" + std::to_string(_error_code) + "] General internal incompatibility error: General internal incompatibility in device";
      case 0x60600000:
         return "[" + std::to_string(_error_code) + "] Hardware error: Access failed due to a hardware error";
      case 0x60700010:
         return "[" + std::to_string(_error_code) + "] Service parameter error: Data type does not match, length or service parameter does not match";
      case 0x60700012:
         return "[" + std::to_string(_error_code) + "] Service parameter too high: Data typ edoes not match, length or service parameter too high";
      case 0x60700013:
         return "[" + std::to_string(_error_code) + "] Service parameter too low: Data type does not match, length or service parameter too low";
      case 0x60900011:
         return "[" + std::to_string(_error_code) + "] Object subindex error: Last read/write command had invalid subindex";
      case 0x60900030:
         return "[" + std::to_string(_error_code) + "] Value range error: Value range of parameter exceeded";
      case 0x60900031:
         return "[" + std::to_string(_error_code) + "] Value too high: Value of parameter written too high";
      case 0x60900032:
         return "[" + std::to_string(_error_code) + "] Value too low: Value of parameter written too low";
      case 0x60900036:
         return "[" + std::to_string(_error_code) + "] Maximum less minimum error: Maximum value is less than minimum value";
      case 0x80000000:
         return "[" + std::to_string(_error_code) + "] General error: ¯\\_(ツ)_/¯ this sucks I guess";
      case 0x80000020:
         return "[" + std::to_string(_error_code) + "] Transfer/store error: Data cannot be transferred/stored";
      case 0x80000021:
         return "[" + std::to_string(_error_code) + "] Local control error: Data cannot transferred/stored to application because of local control";
      case 0x80000022:
         return "[" + std::to_string(_error_code) + "] Wrong device state: Data cannot be transferred/stored to application because of present device state";
      case 0x0F00FFB9:
         return "[" + std::to_string(_error_code) + "] CAN ID error: Wrong CAN ID";
      case 0x0F00FFBC:
         return "[" + std::to_string(_error_code) + "] Service mode error: Device is not in service";
      case 0x0F00FFBE:
         return "[" + std::to_string(_error_code) + "] Password error: Password is wrong ¯\\_(ツ)_/¯";
      case 0x0F00FFBF:
         return "[" + std::to_string(_error_code) + "] Illegal command: RS232 command is illegal (does not exist)";
      case 0x0F00FFC0:
         return "[" + std::to_string(_error_code) + "] Wrong NMT state: Device is in wrong NMT state";
      default:
         return "INVALID ERROR CODE: " + std::to_string(_error_code);
      }
   }

}