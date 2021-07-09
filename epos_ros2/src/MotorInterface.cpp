/*
   ROS Interface for EPOS nodes
   MotorInterface.hpp
   Purpose: Allow EPOS nodes to communicate with ROS

   @author Jared Beard
   @version 1.0 6/4/21
 */

#include <epos_ros2/MotorInterface.hpp>

MotorInterface::MotorInterface(std::string _node_name) : Node(_node_name)
{
	// PARAM INITILIZATION ------------------------------------------------------------------------
	//Publishers
	motor_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/epos_motor_state", 10);

	//Subscriptions
	motor_command_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
		"/epos_motor_command", 1, std::bind(&MotorInterface::motor_callback, this, _1));

	// Spin information
	main_ = this->create_wall_timer(20ms, std::bind(&MotorInterface::main_callback, this));
	interface_ptr_ = new epos2::EPOSWrapper(get_params());
}

void MotorInterface::motor_callback(const sensor_msgs::msg::JointState::SharedPtr _msg)
{
	motor_commands_ = *_msg;
}

void MotorInterface::main_callback()
{

	//for each motor based on control input, execute control command

	//get state

	//publish state
}

epos2::EPOSParams MotorInterface::get_params()
{
	epos2::EPOSParams params;

	// Motors
	std::vector<std::string> motors;
	this->get_parameter("motors/names", motors);
	std::vector<int> ids;
	this->get_parameter("motors/ids", ids);
	for (std::vector<int>::size_type i = 0; i < ids.size(); ++i)
	{
		params.motor_name_map.insert( std::make_pair(motors[i], ids[i]) );
		params.motor_inds.push_back(i);
		params.motor_ind_map.insert( std::make_pair(ids[1],i) );
	}
	this->get_parameter("motors/gear_ratio", params.gear_ratio);
	this->get_parameter("motors/counts_per_rev", params.counts_per_rev);
	this->get_parameter("motors/wheel_radius/cm", params.wheel_radius);
	this->get_parameter("motors/kT", params.kT);
	this->get_parameter("motors/user_limits/ang_vel_rpm", params.ang_vel_limit);
	this->get_parameter("motors/user_limits/acc_rpm", params.acc_limit);
	this->get_parameter("motors/absolute_limits/curr_stall_a", params.stall_current);
	this->get_parameter("motors/user_limits/curr_inst_a", params.instantaneous_current_limit);
	this->get_parameter("motors/user_limits/curr_cont_a", params.continuous_current_limit);

	// EPOS Modules
	this->get_parameter("epos_modules/device", params.device_name);
	this->get_parameter("epos_modules/protocol", params.protocol_stack_name);
	this->get_parameter("epos_modules/com_interface", params.interface_name);
	this->get_parameter("epos_modules/port", params.port_name);
	this->get_parameter("epos_modules/baud_rate", params.baud_rate);

	return params;
}