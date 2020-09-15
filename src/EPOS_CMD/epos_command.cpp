/*
   Maxon Motor Controller eposCommand
   epos_command.cpp
   Purpose: Wrap EPOS commands and integrate with ROS for general purpose motor control

   @author Jared Beard
   @version 1.0 11/13/18
 */
#include <ros/ros.h>
#include <ros/console.h>
#include <Definitions.h>
#include <epos_ros/epos_command.h>
#include <geometry_msgs/Twist.h>

#include <iostream>
#include <string.h>
#include <sstream>
#include <unistd.h>
#include <getopt.h>
#include <stdlib.h>
#include <stdio.h>
#include <list>
#include <math.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/times.h>
#include <sys/time.h>

/////////////////////////////////////////////////////////////////////
/***************************INITIALIZATION**************************/
/////////////////////////////////////////////////////////////////////
/**
    Opens device and subdevices

    @return Success(0)/Failure(1) of commands
 */
int eposCommand::openDevices()
{
		//Success of code
		int result = MMC_FAILED;
		// Internal use of motor parameters
		char* device_name = new char[255];
		char* protocol_stack_name = new char[255];
		char* interface_name = new char[255];
		char* port_name = new char[255];

		strcpy(device_name, device_name_.c_str());
		strcpy(protocol_stack_name, protocol_stack_name_.c_str());
		strcpy(interface_name, interface_name_.c_str());
		strcpy(port_name, port_name_.c_str());

		//std::cout << "dev " << pDeviceName << "__Prot " << pProtocolStackName << "__Int " << pInterfaceName << "__Por" << pPortName << "__Code" << error_code_ << std::endl;

		ROS_INFO("Open device...");
		//Opens device
		key_handle_ = VCS_OpenDevice(device_name, protocol_stack_name, interface_name, port_name, &error_code_);
		//checking device opened
		if(key_handle_!=0 && error_code_ == 0)
		{
				unsigned int baud_rate = 0;
				unsigned int timeout = 0;

				if(VCS_GetProtocolStackSettings(key_handle_, &baud_rate, &timeout, &error_code_))
				{
						if(VCS_SetProtocolStackSettings(key_handle_, baud_rate_, timeout, &error_code_))
						{
								if(VCS_GetProtocolStackSettings(key_handle_, &baud_rate, &timeout, &error_code_))
								{
										if(baud_rate_==(int)baud_rate)
										{
												result = MMC_SUCCESS;
												ROS_INFO("Device opened");
										}
								}
						}
				}
		}
		else
		{
				key_handle_ = 0;
		}
		// remove temporary device std::strings
		delete []device_name;
		delete []protocol_stack_name;
		delete []interface_name;
		delete []port_name;

		//Get initial device state here

		return result;
}

/**
    Closes device and subdevices

    @return Success(0)/Failure(1) of command
 */
int eposCommand::closeDevices()
{
		int result = MMC_FAILED;
		error_code_ = 0;
		ROS_INFO("Close device");

		if(VCS_CloseAllDevices(&error_code_) && error_code_ == 0)
		{
				result = MMC_SUCCESS;
		}
		return result;
}

/////////////////////////////////////////////////////////////////////
/***************************CONFIGURATION***************************/
/////////////////////////////////////////////////////////////////////
/**
    Sets mode for select devices

    @param _ids IDs for devices to set mode
    @param _mode operation mode to be set
    @return Success(0)/Failure(1) of command
 */
int eposCommand::setMode(std::vector<int> _ids, OpMode _mode)
{
		current_mode_ = _mode;
		for (int i = 0; i < _ids.size(); ++i)
		{
				//std::cout << IDs.size() << std::endl;
				if(VCS_SetOperationMode(key_handle_, _ids[i], _mode, &error_code_))
				{
						ROS_INFO("Operation mode set");
				} else
				{
						logError("SetOperationMode");
						return MMC_FAILED;
				}
		}
		return MMC_SUCCESS;
}

/**
    ROS callback for setting operational mode

    @param msg ROS msg of custom type:
 */
/**void eposCommand::setModeCallback(const )
   {
   for(int i = 0; i < msg.nodeID.size(); ++i)
   {
    if (!setMode(msg.nodeID[i],msg.mode, error_code_))
    {
      ROS_ERROR("FAILED TO SET MODE OF NODE %d", msg.nodeID[i]);
      break;
    }
   }
   }

   /**
    Resets device state machine

    @param _node_id node to have operation mode modified

    @return Success(0)/Failure(1) of command
 */
int eposCommand::resetDevice(unsigned short _node_id)
{
		int result = MMC_FAILED;

		if (VCS_ResetDevice(key_handle_, _node_id, &error_code_))
		{
				result = MMC_SUCCESS;
		}

		return result;
}

/**
    Sets Device State in state machine

    @param _node_id node to have operation mode modified
    @param _state desried state of state machine
    @return Success(0)/Failure(1) of command
 */
int eposCommand::setState(unsigned short _node_id, DevState _state)
{
		if( current_state_ == _state || VCS_SetState(key_handle_,_node_id,_state,&error_code_))
		{
				return MMC_SUCCESS;
		} else
		{
				return MMC_FAILED;
		}
}

/**
    Gets Device State in state machine

    @param _node_id node to have operation mode modified
    @param _state state of state machine
    @return Success(0)/Failure(1) of command
 */
int eposCommand::getState(unsigned short _node_id, DevState &_state)
{
		short unsigned int state_value; // = getDevStateValue(state);
		ROS_DEBUG("Retrieving State");
		if( VCS_GetState(key_handle_,_node_id,&state_value,&error_code_))
		{
				ROS_DEBUG("State Retrieved");
				_state = getDevState(state_value);
				//std::cout << "Motor "<< nodeID << " is in state " << state << std::endl;
				return MMC_SUCCESS;

		} else {
				ROS_WARN("State Failed");
				return MMC_FAILED;
		}
}



/////////////////////////////////////////////////////////////////////
/***************************OPERATION*******************************/
/////////////////////////////////////////////////////////////////////

/**
    Get short unsigned integer value for a state

    @param _state state of interest
    @return state integer value
 */
short unsigned int eposCommand::getDevStateValue(DevState _state){
		short unsigned int disabled = 0x0000;
		short unsigned int enabled = 0x0001;
		short unsigned int quickstop = 0x0002;
		short unsigned int fault = 0x0003;

		if (_state == DISABLED) {
				return disabled;
		} else if (_state == ENABLED) {
				return enabled;
		} else if (_state == QUICKSTOP) {
				return quickstop;
		} else if (_state == FAULT) {
				return fault;
		} else {
				std::cout << "Invalid DevState" << std::endl;
				return 8;
		}
}

/**
    Get integer value for a mode

    @param _mode mode of interest
    @return mode integer value
 */
int eposCommand::getModeValue(OpMode _mode){
		int position = 1;
		int velocity = 3;
		int homing = 6;
		int current = -3;

		if (_mode == position) {
				return position;
		} else if (_mode == velocity) {
				return velocity;
		} else if (_mode == homing) {
				return homing;
		} else if (_mode == current) {
				return current;
		} else {
				std::cout << "Invalid OpMode" << std::endl;
				return 8;
		}
}


/**
    Get state for a state integer

    @param _state state of interest
    @return state enum value
 */
enum eposCommand::DevState eposCommand::getDevState(short unsigned int _state)
{
		short unsigned int disabled = 0x0000;
		short unsigned int enabled = 0x0001;
		short unsigned int quickstop = 0x0002;
		short unsigned int fault = 0x0003;

		if (_state == disabled) {
				return DISABLED;
		} else if (_state == enabled) {
				return ENABLED;
		} else if (_state == quickstop) {
				return QUICKSTOP;
		} else if (_state == fault) {
				return FAULT;
		} else {
				ROS_WARN("Invalid DevStateValues");
				return FAULT;
		}
}

/**
    Clears fault for specified motor

    @param _id motor id
    @return Success(0)/Failure(1) of command
 */
int eposCommand::handleFault(int _id)
{
		BOOL is_fault = 0;
		std::cout << "HF start" << std::endl;
		if(VCS_GetFaultState(key_handle_, _id, &is_fault, &error_code_ ))
		{
				if(is_fault)
				{
						logError("VCS_GetFaultState");
						if(VCS_ClearFault(key_handle_, _id, &error_code_) )
						{
								ROS_INFO("Fault Cleared Motor: %b", _id);
								return MMC_SUCCESS;
						} else
						{
								logError("VCS_ClearFault");
								ROS_INFO("Failed Fault Clear");
								return MMC_FAILED;
						}
				} else
				{
						ROS_DEBUG("No fault motor");
				}
		} else
		{
				ROS_WARN("Get Fault state failed");
		}
}

/**
    Enables Specified motors

    @param _ids motor ids
    @return Success(0)/Failure(1) of command
 */
int eposCommand::enableMotors(std::vector<int> _ids)
{
		DevState state;
		for (int i = 0; i < _ids.size(); ++i)
		{
				//std::cout << "Prepare Motors "<< std::endl;
				if (getState(_ids[i], state))
				{
						//std::cout << "State " << state << " ;P" << std::endl;
						if (state == FAULT)
						{
								std::cout << "FAULT" << std::endl;
								handleFault(_ids[i]);
								std::cout << "Clear Fault " << _ids[i] << std::endl;
						}
						if (state != ENABLED)
						{
								setState(_ids[i], ENABLED);
						}
				} else
				{
						ROS_WARN("Get state failed: motor %d", _ids[i]);
				}
		}
		return MMC_SUCCESS;
}



/************


NEED TO CHECK VELOCITY CONVERSION


*************/
/**
    Sets velocity for a list of motors

    @param _ids motor ids
		@param _velocities angular velocities for each motor. If only one velocity listed, it will be applied to all motor ids provided.
    @return Success(0)/Failure(1) of command
 */
int eposCommand::goToVel(std::vector<int> _ids, std::vector<long> _velocities)
{
		if (current_mode_ != OMD_PROFILE_VELOCITY_MODE) setMode(_ids, OMD_PROFILE_VELOCITY_MODE);

		for (int i = 0; i < _ids.size(); ++i)
		{
				if (_velocities.size() < _ids.size() && i > 0) _velocities.push_back(_velocities[0]);

				if (abs(_velocities[i]) > 0)
				{
						if (!VCS_MoveWithVelocity(key_handle_, _ids[i], _velocities[i],&error_code_))
						{
								logError("VCS_MoveWithVelocity");
								return MMC_FAILED;
						} else {
								ROS_INFO("Running");
						}
				} else
				{
						if (!VCS_HaltVelocityMovement(key_handle_, _ids[i],&error_code_))
						{
								return MMC_FAILED;
						}
				}
		}
		return MMC_SUCCESS;
}

/**
    Gets position for a list of motors

    @param _ids motor ids
		@param _positions angular position for each motor
    @return Success(0)/Failure(1) of command
 */
int eposCommand::getPosition(std::vector<int> _ids, std::vector<int> &_positions) //128 cts/turn
{
		int pos = 0;
		//ROS_WARN("---------------------------------------------------%d", pos);
		//*pos = 1;
		//ROS_WARN("---------------------------------------------------%d", *pos);
		for (int i = 0; i < _ids.size(); ++i)
		{
				ROS_WARN("pos %d",  _ids[i]);
				if (VCS_GetPositionIs(key_handle_, _ids[i], &pos, &error_code_))
				{
						ROS_WARN(" is %d", pos);
						_positions.push_back(pos);
						std::cout << " is " << pos << std::endl;
				}
				else
				{
						std::cout << " FAILED POSITION. " << std::endl;
						return MMC_FAILED;
				}
		}

		return MMC_SUCCESS;
}

/**
    Gets current for a list of motors

    @param _ids motor ids
		@param current current for each motor
    @return Success(0)/Failure(1) of command
 */
int eposCommand::getCurrent(std::vector<int> _ids, std::vector<short> &_currents)
{
    	short current = 0;
		//ROS_WARN("---------------------------------------------------%d", pos);
		//*pos = 1;
		//ROS_WARN("---------------------------------------------------%d", *pos);
		for (int i = 0; i < _ids.size(); ++i)
		{
				ROS_WARN("current %d",  _ids[i]);
				if (VCS_GetCurrentIs(key_handle_, _ids[i], &current, &error_code_))
				{
						ROS_WARN(" is %d", current);
						_currents.push_back(current);
						std::cout << " is " << current << std::endl;
				}
				else
				{
						std::cout << " FAILED CURRENT. " << std::endl;
						return MMC_FAILED;
				}
		}

		return MMC_SUCCESS;
}

int eposCommand::goToTorque(std::vector<int> _ids, std::vector<long> _torques, double _gr)
{
		if (current_mode_ != OMD_CURRENT_MODE) setMode(_ids, OMD_CURRENT_MODE);

		for (int i = 0; i < _ids.size(); ++i)
		{
				short current_amps = floor(_torques[i]/(kT_*_gr));
				if (VCS_SetCurrentMust(key_handle_, _ids[i], current_amps,&error_code_))
				{
						ROS_INFO("Running Current");
				} else {
						logError("VCS_SetCurrentMust");
						return MMC_FAILED;
				}

		}
		return MMC_SUCCESS;
}


/////////////////////////////////////////////////////////////////////
/***************************PRINT/DEBUGGING*************************/
/////////////////////////////////////////////////////////////////////
/**
    Displays Error info for an executed function

    @param error_code_Value Error code number
    @return Success(0)/Failure(1) of command
 */
int eposCommand::getError(unsigned short _error_code_value)
{
		int result = MMC_FAILED;
		if(VCS_GetErrorInfo(_error_code_value, error_code_char_, MMC_MAX_LOG_MSG_SIZE))
		{
				ROS_ERROR("ERROR %u: %u\n", _error_code_value, *error_code_char_);
				result = MMC_SUCCESS;
		}

		return result;
}


void eposCommand::logError(std::string _function_name)
{
		std::cerr << "EPOS COMMAND: " << _function_name << " failed (error_code_=0x" << std::hex << error_code_ << ")"<< std::endl;
}


int eposCommand::checkNodeID(int _id)
{
		int result = MMC_FAILED;

		for (int i = 0; i < node_id_list_.size(); ++i)
		{
				if (_id == node_id_list_[i]) {
						result == MMC_SUCCESS;
				}
		}
		return result;
}
/////////////////////////////////////////////////////////////////////
/***************************CONSTRUCTORS****************************/
/////////////////////////////////////////////////////////////////////
/**
    Default Constructor
 */
eposCommand::eposCommand(){
		node_id_list_.push_back(2);
		device_name_ = "EPOS4";
		protocol_stack_name_ = "MAXON SERIAL V2";
		interface_name_ = "USB";
		port_name_ = "USB0";
		baud_rate_ = 1000000;
		num_devices_ = 1;

		ROS_INFO("EPOS COMMAND START");
}

/**
    Constructor for specific motor ids and baudrate

    @param ids id number of motors to be used
    @param br baudrate for communications
 */
eposCommand::eposCommand(std::vector<int> _ids, int _br){
		for (int i = 0; i < _ids.size(); ++i) {
				node_id_list_.push_back( (unsigned short) _ids[i]);

		}
		device_name_ = "EPOS4";
		protocol_stack_name_ = "MAXON SERIAL V2";
		interface_name_ = "USB";
		port_name_ = "USB0";
		baud_rate_ = _br;
		num_devices_ = _ids.size();

		ROS_INFO("EPOS COMMAND START");
}

/**
    Destructor closes all devices
 */
eposCommand::~eposCommand()
{
		int i = 0;
		while(!closeDevices() && i < 5)
		{
				std::cerr << "Failed to close devices, try " << i << "."<< std::endl;
				++i;
		};
		if (i == 5) {
				std::cerr << "Failed: Aborting device closure" << std::endl;
		}
}
