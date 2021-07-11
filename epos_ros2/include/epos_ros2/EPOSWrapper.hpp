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

#ifndef MAX_LOG_MSG_SIZE
#define MAX_LOG_MSG_SIZE 512
#endif

#define assertm(exp, msg) assert(((void)msg, exp))

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

#include <Definitions.h>

#include <rclcpp/rclcpp.hpp>

// #include <geometry_msgs/msg/Twist.h>

#include <epos_ros2/utils/EPOSParams.hpp>
//#include <epos_ros2/ROSNodeParams.hpp>

typedef void *HANDLE;
typedef int BOOL;
typedef unsigned long DWORD;

namespace epos2
{

	class EPOSWrapper
	{
		/////////////////////////////////////////////////////////////////////
   		/***************************INITIALIZER LIST************************/
   		/////////////////////////////////////////////////////////////////////

		// void *key_handle_ = 0;
		// DWORD error_code_ = 0;
		// //unsigned short maxStrSize = 512;
		// char *error_code_char_;

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
		//~EPOSWrapper();

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
		// int setMode(std::vector<int> _node_ids, OpMode _mode);
		// //void setModeCallback(const ; //NEED TO MAKE MESSAGE FOR THIS, need to make way to display/handle specific error
		// int resetDevice(unsigned short _node_id);
		// int setState(unsigned short _node_id, DevState _state);
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

		/////////////////////////////////////////////////////////////////////
    /**************************ERRORS and LOGGING***********************/
    /////////////////////////////////////////////////////////////////////

		/***********************VARIABLES*****************************/
		// std::vector<int> current_state_;
		// std::vector<int> current_mode_;
		//ROSNodeParams node_params_;
		rclcpp::Node *node_ptr_;
		EPOSParams epos_params_;

		//Setup error codes to print intead of being accepted as input. Makes the output simpler...
		/***********************INTIALIZATION***************************/
		int open_devices();
		// int close_devices();

		
		


	private:
		/////////////////////////////////////////////////////////////////////
   		/***************************VARIABLES*******************************/
   		/////////////////////////////////////////////////////////////////////

		// /***********************OPERATION*****************************/
		// short unsigned int getDevStateValue(DevState state);
		// enum DevState getDevState(short unsigned int state);
		// int getModeValue(OpMode mode);

		//  int   DemoProfilePositionMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode);
		//  int   Demo(unsigned int* pErrorCode);
		//  int   PrepareDemo(unsigned int* pErrorCode);

		//  void cmdReceived(const geometry_msgs::Twist& msg);
		//  void clearFaultCallback(const sensor_msgs::Joy& msg);
		

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
		void log_msg(std::string _msg, loggingVerbosity _verbosity, loggingGroup _group);

		/**
       	 * Converts error codes to description of error as provided in Maxon
		 * communication library
		 * @brief Gets error description
       	 * @param _error_code Error code number
         * @return Error description
    	*/
		std::string get_error_code(DWORD _error_code);
		

		//  int   PrepareMotor(unsigned int* pErrorCode, unsigned short int nodeId);
	};
}
#endif

