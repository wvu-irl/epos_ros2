/*
   Maxon Motor Controller eposCommand
   epos_command.h
   Purpose: Wrap EPOS commands and integrate with ROS for general purpose motor control

   @author Jared Beard
   @version 1.0 11/13/18
 */
#ifndef EPOS_WRAPPER_HPP
#define EPOS_WRAPPER_HPP

#ifndef RETURN_SUCCESS
#define RETURN_SUCCESS 1
#endif

#ifndef RETURN_FAILED
#define RETURN_FAILED 0
#endif

// #ifndef MAX_LOG_MSG_SIZE
// #define MAX_LOG_MSG_SIZE 512
// #endif

#define assertm(exp, msg) assert(((void)msg, exp))

// C++ libraries
#include <cassert>
#include <string>
// #include <iostream>
// #include <sstream>
// #include <unistd.h>
// #include <getopt.h>
// #include <stdlib.h>
// #include <stdio.h>
// #include <list>
// #include <math.h>
// #include <sys/types.h>
// #include <unistd.h>
// #include <sys/times.h>
// #include <sys/time.h>

// EPOS C Communication Library
#include <Definitions.h>

// ROS2 Libraries
#include <rclcpp/rclcpp.hpp>

// Custom Libraries
#include <epos_ros2/utils/EPOSParams.hpp>

typedef void *HANDLE;
typedef int BOOL;
typedef unsigned int DWORD;
typedef unsigned short WORD;

namespace epos2
{

	class EPOSWrapper
	{
		/////////////////////////////////////////////////////////////////////
		/***************************INITIALIZER LIST************************/
		/////////////////////////////////////////////////////////////////////

		/// maximum size of string containing error information, set to 512
		WORD max_log_size_ = 512;
		/// Maps DeviceState to short unsigned int*
		std::map<WORD, DeviceState> device_state_map_ = {{0x0000, DISABLED}, {0x0001, ENABLED}, {0x0002, QUICKSTOP}, {0x0003, FAULT}};

	public:
		/////////////////////////////////////////////////////////////////////
		/***************************VARIABLES*******************************/
		/////////////////////////////////////////////////////////////////////

		/////////////////////////////////////////////////////////////////////
		/***************************CONSTRUCTORS****************************/
		/////////////////////////////////////////////////////////////////////

		/**
		 * Asserts Error: default constructor doesn't make sense without ROS2 node
    	 * @brief Default Constructor
   		*/
		EPOSWrapper();

		/**
    	 * Constructs wrapper object for a ROS2 node and with parameters specified by epos2::EPOSParams struct
		 * @brief Special Constructor
    	 * @param _node Pointer to a ROS2 node
    	 * @param _epos_params Struct of EPOS parameters
   		*/
		EPOSWrapper(rclcpp::Node *_node_ptr, EPOSParams _epos_params);

		/**
         * Destructs object and attempts to close all devices
		 * @brief Destructor
    	*/
		~EPOSWrapper();

		/////////////////////////////////////////////////////////////////////
		/***************************INITIALIZATION**************************/
		/////////////////////////////////////////////////////////////////////

		/**
       	 * Requires that device_name, protocol_stack_name, interface_name and port_name be 
		 * set in mac::EPOSParams struct 
		 * 
         * @brief Opens device and subdevices
         * @return Success(1)/Failure(0) of commands
     	*/
		int open_devices();

		/**
		 * @brief Closess device and subdevices
    	 * @return Success(1)/Failure(0) of command
    	*/
		int close_devices();

		/////////////////////////////////////////////////////////////////////
		/***************************CONFIGURATION***************************/
		/////////////////////////////////////////////////////////////////////

		/**
		 * 
		 * @brief Sets the mode for a vector of motors
		 * @param _motors motor names for devices to set mode
		 * @param _mode operation mode to be set
		 * @return Success(1)/Failure(0) of command
    	*/
		int set_modes(std::vector<std::string> _motor, std::vector<OperationMode> _modes);

		/**
       	 * All motors will be set to the same mode
		 * 
		 * @brief Sets the mode for a vector of motors
		 * @param _motors motor names for devices to set mode
		 * @param _mode operation mode to be set
		 * @return Success(1)/Failure(0) of command
    	*/
		int set_modes(std::vector<std::string> _motor, OperationMode _mode);

		/**
		 * 
		 * @brief Sets the mode for an individual motors
		 * @param _motor motor name for devices to set mode
		 * @param _mode operation mode to be set
		 * @return Success(1)/Failure(0) of command
    	*/
		int set_mode(std::string _motor, OperationMode _mode);

		/**
		 *
		 * @brief Resets device state machine for those specified
		 * @param _motors motor to have operation mode reset
       	 * @return Success(1)/Failure(0) of command
    	*/
		int reset_devices(std::vector<std::string> _motors);

		/**
		 *
		 * @brief Resets device state machine for that specified
		 * @param _motors motor to have operation mode reset
       	 * @return Success(1)/Failure(0) of command
    	*/
		int reset_device(std::string _motor);

		/**
    	 * 
    	 * @brief Sets Device State in state machine
    	 * @param _motors Motors to set device state of
    	 * @param _state Desired states of state machines
    	 * @return Success(1)/Failure(0) of command
   		*/
		int set_states(std::vector<std::string> _motors, std::vector<DeviceState> _states);

		/**
    	 * 
    	 * @brief Sets Device State in state machine
    	 * @param _motors Motors to set device state of
    	 * @param _state Desired state of state machines
    	 * @return Success(1)/Failure(0) of command
   		*/
		int set_states(std::vector<std::string> _motors, DeviceState _state);

		/**
    	 * 
    	 * @brief Sets Device State in state machine
    	 * @param _motor Motor to set device state of
    	 * @param _state Desired state of state machine
    	 * @return Success(1)/Failure(0) of command
   		*/
		int set_state(std::string _motor, DeviceState _state);

		/**
    	 * 
    	 * @brief Gets Device States passed back through reference
    	 * @param _motors Motor to get device state of
    	 * @param _states State of state machine
    	 * @return Success(1)/Failure(0) of command
   		*/
   		int get_states(std::vector<std::string> _motors, std::vector<DeviceState> &_states);

		/**
    	 * 
    	 * @brief Gets Device States passed back through reference
    	 * @param _motor Motor to set device data of
    	 * @param _state State of state machine
    	 * @return Success(1)/Failure(0) of command
   		*/
   		int get_state(std::string _motor, DeviceState &_state);

		/////////////////////////////////////////////////////////////////////
		/***************************OPERATION*******************************/
		/////////////////////////////////////////////////////////////////////

		/////////////////////////////////////////////////////////////////////
		/**************************ERRORS and LOGGING***********************/
		/////////////////////////////////////////////////////////////////////

		/**
       	 * Converts error codes to description of error as provided in Maxon
		 * communication library. Includes device errors
		 * @brief Gets error description
       	 * @param _error_code Error code number
         * @return Error description
    	*/
			static std::string get_error(DWORD _error_code);

		/**
       	 * Converts error codes to description of error as provided in Maxon
		 * communication library. Does not invlude device errors
		 * @brief Gets error description from device
       	 * @param _error_code Error code number
         * @return Error description
    	*/
		std::string get_error_vcs(DWORD _error_code);

		/**
       	 * 
		 * @brief Gets state string description enum value
       	 * @param _state Device state to convert to string
         * @return Device state string
    	*/
		std::string get_state_string(DeviceState _state);

		/**
       	 * 
		 * @brief Gets mode string description enum value
       	 * @param _state Device mode to convert to string
         * @return Device mode string
    	*/
		std::string get_mode_string(OperationMode _mode);

	private:
		/////////////////////////////////////////////////////////////////////
		/***************************VARIABLES*******************************/
		/////////////////////////////////////////////////////////////////////

		/// Pointer to ROS2 node
		rclcpp::Node *node_ptr_;
		/// Parameters for EPOS device, motors, and logging
		EPOSParams epos_params_;
		/// Handle for port access
		HANDLE key_handle_ = 0;

		/////////////////////////////////////////////////////////////////////
		/***************************CONFIGURATION***************************/
		/////////////////////////////////////////////////////////////////////

		/////////////////////////////////////////////////////////////////////
		/***************************OPERATION*******************************/
		/////////////////////////////////////////////////////////////////////

		/////////////////////////////////////////////////////////////////////
		/**************************ERRORS and LOGGING***********************/
		/////////////////////////////////////////////////////////////////////

		/**
		 * Wraps ROS2 logging and adds additional semantic behaviors
		 * @brief ROS2 logging wrapper
    	 * @param _msg Error message
    	 * @param _verbosity Level of verbosity
    	 * @param _group Group of log message 
    	*/
		void log_msg(std::string _msg, loggingVerbosity _verbosity = LOG_OFF, loggingGroup _group = GROUP_ERROR);
	};
}
#endif

//  int   PrepareMotor(unsigned int* pErrorCode, unsigned short int nodeId);

// /***********************OPERATION*****************************/
// short unsigned int getDevStateValue(DevState state);
// enum DevState getDevState(short unsigned int state);
// int getModeValue(OpMode mode);

//  int   DemoProfilePositionMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode);
//  int   Demo(unsigned int* pErrorCode);
//  int   PrepareDemo(unsigned int* pErrorCode);

//  void cmdReceived(const geometry_msgs::Twist& msg);
//  void clearFaultCallback(const sensor_msgs::Joy& msg);

/**   // get
   int   setPosProfile();
   int   goToPos();
   int   stopPos();
   int   setVelProfile();
   int   startHomingMode();
   int   setHome();
   int   goToHome();
   int   stopHome();
   int   waitForHome();
   int   setCurrentMust();
   int   getCurrentMust();*/
// getpositionis -> encoder?? try to find something that decouples these from the hall sensors
//getvelocityis
//get currentis
//waitfortargetreached

// /***********************CONFIGURATION***************************/
// int resetDevice(unsigned short _node_id);
// int getState(unsigned short _node_id, DevState &_state);

// /***********************OPERATION*******************************/
// virtual bool checkFault();
// int handleFault(int _id);
// int enableMotors(std::vector<int> _ids);
// int goToVel(std::vector<int> _ids, std::vector<long> _velocities);
// //int stopVel(std::vector<int> IDs);
// int getPosition(std::vector<int> _ids, std::vector<int> &_positions); //put in check that mode is correct
// int getCurrent(std::vector<int> _ids, std::vector<short> &_currents);
// int goToTorque(std::vector<int> _ids, std::vector<long> _torques, double _gr);

// /***********************PRINT/DEBUGGING*************************/
// int getError(unsigned short _error_code_value);   //NEED to convert error code to text
// void logError(std::string _function_name);
// int checkNodeID(int _id);
// int addNodeIDs(std::vector<int> _ids);
