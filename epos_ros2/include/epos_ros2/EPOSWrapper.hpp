/**
 * @file EPOSWrapper.hpp
 * @author  Jared J Beard <jbeard6@mix.wvu.edu>
 * @version 1.0
 *
 * @section DESCRIPTION
 *
 * Wraps core EPOS functionalities, does not explicity handle faults
 * and communicate with core system
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
#include <epos_ros2/EPOSParams.hpp>

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
         * @param _params Struct of EPOS parameters
   		*/
        EPOSWrapper(rclcpp::Node *_node_ptr, EPOSParams _params);

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
        int set_modes(const std::vector<std::string> &_motors,const std::vector<OperationMode> &_modes);

        /**
         * All motors will be set to the same mode
         * 
         * @brief Sets the mode for a vector of motors
         * @param _motors motor names for devices to set mode
         * @param _mode operation mode to be set
         * @return Success(1)/Failure(0) of command
    	*/
        int set_modes(const std::vector<std::string> &_motors, OperationMode _mode);

        /**
         * 
         * @brief Sets the mode for an individual motors
         * @param _motor motor name for devices to set mode
         * @param _mode operation mode to be set
         * @return Success(1)/Failure(0) of command
    	*/
        int set_mode(const std::string &_motor, OperationMode _mode);

        /**
         *
         * @brief Resets device state machine for those specified
         * @param _motors motor to have operation mode reset
         * @return Success(1)/Failure(0) of command
    	*/
        int reset_devices(const std::vector<std::string> &_motors);

        /**
         *
         * @brief Resets device state machine for that specified
         * @param _motors motor to have operation mode reset
         * @return Success(1)/Failure(0) of command
    	*/
        int reset_device(const std::string &_motor);

        /**
    	 * 
    	 * @brief Sets Device State in state machine
    	 * @param _motors Motors to set device state of
    	 * @param _state Desired states of state machines
    	 * @return Success(1)/Failure(0) of command
   		*/
        int set_states(const std::vector<std::string> &_motors, const std::vector<DeviceState> &_states);

        /**
    	 * 
    	 * @brief Sets Device State in state machine
    	 * @param _motors Motors to set device state of
    	 * @param _state Desired state of state machines
    	 * @return Success(1)/Failure(0) of command
   		*/
        int set_states(const std::vector<std::string> &_motors, DeviceState _state);

        /**
    	 * 
    	 * @brief Sets Device State in state machine
    	 * @param _motor Motor to set device state of
    	 * @param _state Desired state of state machine
    	 * @return Success(1)/Failure(0) of command
   		*/
        int set_state(const std::string &_motor, DeviceState _state);

        /**
    	 * 
    	 * @brief Gets Device States passed back through reference
    	 * @param _motors Motor to get device state of
    	 * @param _states State of state machine
    	 * @return Success(1)/Failure(0) of command
   		*/
        int get_states(const std::vector<std::string> &_motors, std::vector<DeviceState> &_states);

        /**
    	 * 
    	 * @brief Gets Device States passed back through reference
    	 * @param _motor Motor to set device data of
    	 * @param _state State of state machine
    	 * @return Success(1)/Failure(0) of command
   		*/
        int get_state(const std::string &_motor, DeviceState &_state);

        /**
         * 
         * @brief Clears faults of selected motors
         * @param _motors names of motors to clear faults for
         * @return Success(1)/Failure(0) of command
    	*/
        int clear_faults(const std::vector<std::string> &_motors);

        /**
         * 
         * @brief Clears faults of selected motor
         * @param _motors name of motor to clear fault for
         * @return Success(1)/Failure(0) of command
    	*/
        int clear_fault(const std::string &_motor);

        /////////////////////////////////////////////////////////////////////
        /***************************OPERATION*******************************/
        /////////////////////////////////////////////////////////////////////

        /**
    	 * 
    	 * @brief Command motors to go to a list of velocities
    	 * @param _motor Motors to command velocities
    	 * @param _velocities Desired motor velocities
         * @param _rpm if true, units are in RPM else m/s (default: true) note, double gets casted to long
    	 * @return Success(1)/Failure(0) of command
   		*/
        int go_to_velocities(const std::vector<std::string> &_motors, const std::vector<double> &_velocities, bool _rpm);

        /**
    	 * 
    	 * @brief Command motors to go to a velocity
    	 * @param _motor Motors to command velocity
    	 * @param _velocity Desired motor velocities
         * @param _rpm if true, units are in RPM else m/s (default: true) note, double gets casted to long
    	 * @return Success(1)/Failure(0) of command
   		*/
        int go_to_velocities(const std::vector<std::string> &_motors, double _velocity, bool _rpm);

        /**
    	 * 
    	 * @brief Command motor to go to a velocity
    	 * @param _motor Motor to command velocity
    	 * @param _velocity Desired motor velocity
         * @param _rpm if true, units are in RPM else m/s (default: true) note, double gets casted to long
    	 * @return Success(1)/Failure(0) of command
   		*/
        int go_to_velocity(const std::string &_motor, double _velocity, bool _rpm);

        /**
    	 * 
    	 * @brief Get velocities of motors
    	 * @param _motor Motors to get velocities
    	 * @param _velocities Current motor velocities
         * @param _rpm if true, units are in RPM else m/s (default: true)
    	 * @return Success(1)/Failure(0) of command
   		*/
        int get_velocities(const std::vector<std::string> &_motors, std::vector<double> &_velocities, bool _rpm);

        /**
    	 * 
    	 * @brief Get velocity of motors
    	 * @param _motor Motors to velocity
    	 * @param _velocity Current motor velocity
         * @param _rpm if true, units are in RPM else m/s (default: true)
    	 * @return Success(1)/Failure(0) of command
   		*/
        int get_velocity(const std::string &_motor, double &_velocity, bool _rpm);

        /**
    	 * 
    	 * @brief Command motors to go to a list of positions
    	 * @param _motor Motors to command positions
    	 * @param _positions Desired motor positions
         * @param _counts if true, units are in encoder counts else m (default: true) note, double gets casted to long
         * @param _absolute if true uses absolute position command, else relative (default: true)
         * @param _immediate if true starts command immediately, else waits until last position command met (default: true)
    	 * @return Success(1)/Failure(0) of command
   		*/
        int go_to_positions(const std::vector<std::string> &_motors, const std::vector<double> &_positions, bool _counts, bool _absolute, bool _immediate);

        /**
    	 * 
    	 * @brief Command motors to go to a position
    	 * @param _motor Motors to command position
    	 * @param _position Desired motor positions
         * @param _counts if true, units are in encoder counts else m (default: true) note, double gets casted to long
         * @param _absolute if true uses absolute position command, else relative (default: true)
         * @param _immediate if true starts command immediately, else waits until last position command met (default: true)
    	 * @return Success(1)/Failure(0) of command
   		*/
        int go_to_positions(const std::vector<std::string> &_motors, double _position, bool _counts, bool _absolute, bool _immediate);

        /**
    	 * 
    	 * @brief Command motor to go to a position
    	 * @param _motor Motor to command position
    	 * @param _position Desired motor position
         * @param _counts if true, units are in encoder counts else m (default: true) note, double gets casted to long
         * @param _absolute if true uses absolute position command, else relative (default: true)
         * @param _immediate if true starts command immediately, else waits until last position command met (default: true)
    	 * @return Success(1)/Failure(0) of command
   		*/
        int go_to_position(const std::string &_motor, double _position, bool _counts, bool _absolute, bool _immediate);

        /**
    	 * 
    	 * @brief Get positions of motors
    	 * @param _motor Motors to get positions
    	 * @param _positions Current motor positions
         * @param _counts if true, units are in encoder counts else m (default: true) note, double gets casted to long
    	 * @return Success(1)/Failure(0) of command
   		*/
        int get_positions(const std::vector<std::string> &_motors, std::vector<double> &_positions, bool _counts);

        /**
    	 * 
    	 * @brief Get position of motors
    	 * @param _motor Motors to position
    	 * @param _position Current motor position
         * @param _counts if true, units are in encoder counts else m (default: true) note, double gets casted to long
    	 * @return Success(1)/Failure(0) of command
   		*/
        int get_position(const std::string &_motor, double &_position, bool _counts);

        /**
    	 * 
    	 * @brief Command motors to go to a list of torques
    	 * @param _motor Motors to command torques
    	 * @param _torques Desired motor torques (mNm) note, double gets converted to short for current in (mA)
    	 * @return Success(1)/Failure(0) of command
   		*/
        int go_to_torques(const std::vector<std::string> &_motors, const std::vector<double> &_torques);

        /**
    	 * 
    	 * @brief Command motors to go to a torque
    	 * @param _motor Motors to command torque
    	 * @param _torque Desired motor torques (mNm) note, double gets converted to short for current in (mA)
    	 * @return Success(1)/Failure(0) of command
   		*/
        int go_to_torques(const std::vector<std::string> &_motors, double _torque);

        /**
    	 * 
    	 * @brief Command motor to go to a torque
    	 * @param _motor Motor to command torque
    	 * @param _torque Desired motor torque (mNm) note, double gets converted to short for current in (mA)
    	 * @return Success(1)/Failure(0) of command
   		*/
        int go_to_torque(const std::string &_motor, double _torque);

        /**
    	 * 
    	 * @brief Get torques of motors
    	 * @param _motor Motors to get torques
    	 * @param _torques Current motor torques (mNm) note, double gets converted to short for current in (mA)
    	 * @return Success(1)/Failure(0) of command
   		*/
        int get_torques(const std::vector<std::string> &_motors, std::vector<double> &_torques);

        /**
    	 * 
    	 * @brief Get torque of motors
    	 * @param _motor Motors to torque
    	 * @param _torque Current motor torque (mNm) note, double gets converted to short for current in (mA)
    	 * @return Success(1)/Failure(0) of command
   		*/
        int get_torque(const std::string &_motor, double &_torque);

        /**
    	 * 
    	 * @brief Get currents of motors
    	 * @param _motor Motors to currents
    	 * @param _currents Currents (mA)
    	 * @return Success(1)/Failure(0) of command
   		*/
        int get_currents(const std::vector<std::string> &_motors, std::vector<double> &_currents);

        /**
    	 * 
    	 * @brief Get current of motor
    	 * @param _motor Motors to current
    	 * @param _current Current (mA)
    	 * @return Success(1)/Failure(0) of command
   		*/
        int get_current(const std::string &_motor, double &_current);

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
        /// Clock for Logger throttling
        rclcpp::Clock clock_;
        /// Parameters for EPOS device, motors, and logging
        EPOSParams params_;
        /// Handle for port access
        HANDLE key_handle_ = 0;

        /////////////////////////////////////////////////////////////////////
        /***************************CONFIGURATION***************************/
        /////////////////////////////////////////////////////////////////////

        /////////////////////////////////////////////////////////////////////
        /***************************OPERATION*******************************/
        /////////////////////////////////////////////////////////////////////

        /**
    	 * 
    	 * @brief Stops motor velocity
    	 * @param _motor Motor to stop
    	 * @return Success(1)/Failure(0) of command
   		*/
        int halt_velocity(const std::string &_motor);

        /**
    	 * 
    	 * @brief Stops motor at position
    	 * @param _motor Motor to stop
    	 * @return Success(1)/Failure(0) of command
   		*/
        int halt_position(const std::string &_motor);

        /////////////////////////////////////////////////////////////////////
        /**************************ERRORS and LOGGING***********************/
        /////////////////////////////////////////////////////////////////////

        /////////////////////////////////////////////////////////////////////
        /**************************UTILS************************************/
        /////////////////////////////////////////////////////////////////////

        /**
    	 * 
    	 * @brief Convert motor velcoty from RPM to m/s
    	 * @param _motor Motor to convert velocities
    	 * @param _velocity Motor velocity (RPM)
    	 * @return Motor velocity (m/s)
   		*/
        double rpm_2_mps(MaxonMotor &_motor, double _velocity);

        /**
    	 * 
    	 * @brief Convert motor velocity from RPM to m/s
    	 * @param _motor Motor to convert velocities
    	 * @param _velocity Motor velocity (m/s)
    	 * @return Motor velocity (RPM)
   		*/
        double mps_2_rpm(MaxonMotor &_motor, int _velocity);

        /**
    	 * 
    	 * @brief Convert motor position from encoder counts to m
    	 * @param _motor Motor to convert position
    	 * @param _position Motor position (encoder counts)
    	 * @return Motor position (m)
   		*/
        double count_2_m(MaxonMotor &_motor, double _position);

        /**
    	 * 
    	 * @brief Convert motor velcoty from encoder counts to m
    	 * @param _motor Motor to convert position
    	 * @param _position Motor position (m)
    	 * @return Motor position (encoder counts)
   		*/
        double m_2_count(MaxonMotor &_motor, double _position);
    };
}
#endif
