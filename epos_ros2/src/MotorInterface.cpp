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
	declare_params();

	//Publishers
	motor_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/epos_motor_state", 10);

	//Subscriptions
	motor_command_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
		"/epos_motor_command", 1, std::bind(&MotorInterface::motor_callback, this, _1));

	// Spin information
	main_ = this->create_wall_timer(20ms, std::bind(&MotorInterface::main_callback, this));
	interface_ptr_ = new epos2::EPOSWrapper(this, get_params());
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

void MotorInterface::declare_params()
{
	// DECLARE PARAMS --------------------------------------------------

	this->declare_parameter("motors/names");
	rclcpp::Parameter motor_names_param("motor/names", std::vector<std::string>{"none"});
	special_params_.push_back(motor_names_param);

	this->declare_parameter("motors/ids");
	rclcpp::Parameter motor_ids_param("motor/ids", std::vector<int>{0});
	special_params_.push_back(motor_ids_param);

	this->declare_parameter("motors/gear_ratio", 1);
	this->declare_parameter("motors/counts_per_rev", 1.0);
	this->declare_parameter("motors/wheel_radius/cm", 1.0);
	this->declare_parameter("motors/kT", 1.0);
	this->declare_parameter("motors/user_limits/ang_vel_rpm", 1.0);
	this->declare_parameter("motors/user_limits/acc_rpm", 1.0);
	this->declare_parameter("motors/absolute_limits/curr_stall_a", 1.0);
	this->declare_parameter("motors/user_limits/curr_inst_a", 1.0);
	this->declare_parameter("motors/user_limits/curr_cont_a", 1.0);

	// EPOS Modules
	this->declare_parameter("epos_modules/device", "EPOS4");
	this->declare_parameter("epos_modules/protocol", "MAXON SERIAL V2");
	this->declare_parameter("epos_modules/com_interface", "USB");
	this->declare_parameter("epos_modules/port", "USB0");
	this->declare_parameter("epos_modules/baud_rate", 1000000);

	// // Logging
	this->declare_parameter("logging/is_on", false);
	this->declare_parameter("logging/groups");
	rclcpp::Parameter log_groups_param("logging/groups", std::vector<int>{0});
	special_params_.push_back(log_groups_param);
}

epos2::EPOSParams MotorInterface::get_params()
{
	epos2::EPOSParams params;
	int special_param_counter = 0;

	// Motors
	std::vector<std::string> motors;
	this->get_parameter("motors/names", special_params_[special_param_counter]);
	motors = special_params_[special_param_counter].as_string_array();
	++special_param_counter;

	std::vector<int> ids;
	this->get_parameter("motors/ids", special_params_[special_param_counter]);
	ids = std::vector<int>(special_params_[special_param_counter].as_integer_array().begin(),
						   special_params_[special_param_counter].as_integer_array().end());
	++special_param_counter;

	for (std::vector<int>::size_type i = 0; i < ids.size(); ++i)
	{
		params.motor_ids.insert(std::make_pair(motors[i], ids[i]));
		params.motor_inds.insert(std::make_pair(motors[i], i));
	}

	// Maxon Motors
	epos2::MaxonMotor temp_motor;
	for (std::vector<int>::size_type i = 0; i < ids.size(); ++i)
	{
		temp_motor.index = ids[i];
		this->get_parameter("motors/gear_ratio", temp_motor.gear_ratio);
		this->get_parameter("motors/counts_per_rev", temp_motor.counts_per_rev);
		this->get_parameter("motors/wheel_radius/cm", temp_motor.wheel_radius);
		this->get_parameter("motors/kT", temp_motor.kT);
		this->get_parameter("motors/user_limits/ang_vel_rpm", temp_motor.ang_vel_limit);
		this->get_parameter("motors/user_limits/acc_rpm", temp_motor.acc_limit);
		this->get_parameter("motors/absolute_limits/curr_stall_a", temp_motor.stall_current);
		this->get_parameter("motors/user_limits/curr_inst_a", temp_motor.instantaneous_current_limit);
		this->get_parameter("motors/user_limits/curr_cont_a", temp_motor.continuous_current_limit);

		params.motors.push_back(temp_motor);
	}

	// EPOS Modules
	this->get_parameter("epos_modules/device", params.device_name);
	this->get_parameter("epos_modules/protocol", params.protocol_stack_name);
	this->get_parameter("epos_modules/com_interface", params.interface_name);
	this->get_parameter("epos_modules/port", params.port_name);
	this->get_parameter("epos_modules/baud_rate", params.baud_rate);

	// // Logging
	this->get_parameter("logging/is_on", params.is_on);
	this->get_parameter("logging/groups", special_params_[special_param_counter]);
	params.groups = std::vector<int>(special_params_[special_param_counter].as_integer_array().begin(),
									 special_params_[special_param_counter].as_integer_array().end());
	++special_param_counter;

	return params;
}