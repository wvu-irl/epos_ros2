/*
   Maxon Motor Controller eposCommand
   epos_command.h
   Purpose: Wrap EPOS commands and integrate with ROS for general purpose motor control

   @author Jared Beard
   @version 1.0 11/13/18
 */
#ifndef MOTOR_CMD_H
#define MOTOR_CMD_H

	#ifndef MMC_SUCCESS
	 #define MMC_SUCCESS 1
	#endif

	#ifndef MMC_FAILED
	 #define MMC_FAILED 0
	#endif

	#ifndef MMC_MAX_LOG_MSG_SIZE
	 #define MMC_MAX_LOG_MSG_SIZE 512
	#endif

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>

#include <iostream>
#include <Definitions.h>
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

typedef void* HANDLE;
typedef int BOOL;

#ifndef MMC_MAX_LOG_MSG_SIZE
	#define MMC_MAX_LOG_MSG_SIZE 512
#endif

class eposCommand {
void* key_handle_ = 0;
unsigned int error_code_ = 0;
//unsigned short maxStrSize = 512;

std::vector<unsigned short> node_id_list_;

std::string device_name_;
std::string protocol_stack_name_;
std::string interface_name_;
std::string port_name_;
int baud_rate_;
int num_devices_;
char* error_code_char_;


public:
/***********************ENUMS*********************************/
enum OpMode {
		OMD_PROFILE_POSITION_MODE = 1,
		OMD_PROFILE_VELOCITY_MODE = 3,
		OMD_HOMING_MODE = 6,
		OMD_CURRENT_MODE = -3
};
enum DevState {
		DISABLED = 0x0000,
		ENABLED = 0x0001,
		QUICKSTOP = 0x0002,
		FAULT = 0x0003
};

/***********************VARIABLES*****************************/
DevState current_state_;
OpMode current_mode_;

//Setup error codes to print intead of being accepted as input. Makes the output simpler...
/***********************INTIALIZATION***************************/
int openDevices   ();
int closeDevices  ();

/***********************CONFIGURATION***************************/
int setMode(std::vector<int> _node_ids, OpMode _mode);
//void setModeCallback(const ; //NEED TO MAKE MESSAGE FOR THIS, need to make way to display/handle specific error
int resetDevice(unsigned short _node_id);
int setState(unsigned short _node_id, DevState _state);
int getState(unsigned short _node_id, DevState &_state);

/***********************OPERATION*******************************/
int handleFault(int _id);
int enableMotors(std::vector<int> _ids);
int goToVel(std::vector<int> _ids, std::vector<long> _velocities);
//int stopVel(std::vector<int> IDs);
int getPosition(std::vector<int> _ids, std::vector<int> &_positions); //put in check that mode is correct
int getCurrent(std::vector<int> _ids, std::vector<short> &_currents);
int goToTorque(std::vector<int> _ids, std::vector<long> _torques, double _gr);

/***********************PRINT/DEBUGGING*************************/
int getError(unsigned short _error_code_value);   //NEED to convert error code to text
void logError(std::string _function_name);
int checkNodeID(int _id);
int addNodeIDs(std::vector<int> _ids);


/***********************CONSTRUCTORS****************************/
eposCommand();   // Set motor type, sensor types, max following error, max velocity, max acc,
// velocity units, default operation mode
eposCommand(std::vector<int> _ids, int _br);
//motor_cmd(); <- read input from launch
~eposCommand();





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



private:
/***********************VARIABLES*****************************/
double kT_ = 31.5; // mNm/A
/***********************OPERATION*****************************/
short unsigned int getDevStateValue(DevState state);
enum DevState getDevState(short unsigned int state);
int getModeValue(OpMode mode);


//  int   DemoProfilePositionMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode);
//  int   Demo(unsigned int* pErrorCode);
//  int   PrepareDemo(unsigned int* pErrorCode);


//  void cmdReceived(const geometry_msgs::Twist& msg);
//  void clearFaultCallback(const sensor_msgs::Joy& msg);

/***************Print and Commands*****************/

//  int   PrepareMotor(unsigned int* pErrorCode, unsigned short int nodeId);

};
#endif
