/*
   ROS Interface for EPOS nodes
   MotorInterface.hpp
   Purpose: Allow EPOS nodes to communicate with ROS

   @author Jared Beard
   @version 1.0 6/4/21
 */

#include <epos_ros2/MotorInterface.hpp>


// TODO: Try opening with bad motor ids


///
///
///
void MotorInterface::motor_callback(const sensor_msgs::msg::JointState::SharedPtr _msg)
{
	motor_commands_ = *_msg;

	drive_motors(*_msg);
}

///
///
///
void MotorInterface::status_callback()
{
	// sensor_msgs::msg::JointState temp;
	// RCLCPP_WARN(this->get_logger(), std::to_string(params_.motor_names.size()).c_str());
	// for (std::string motor : params_.motor_names)
	// {
	// 	temp.name.push_back(motor);
	// 	RCLCPP_WARN(this->get_logger(), motor.c_str());

	// 	if (position_status)
	// 	{
	// 		double position;
	// 		interface_ptr_->get_position(motor, position);
	// 		temp.position.push_back(position);
	// 	}
	// 	else
	// 	{
	// 		temp.position.push_back(-1);
	// 	}

	// 	if (velocity_status)
	// 	{
	// 		double velocity;
	// 		interface_ptr_->get_velocity(motor, velocity);
	// 		temp.velocity.push_back(velocity);
	// 	}
	// 	else
	// 	{
	// 		temp.velocity.push_back(-1);
	// 	}

	// 	if (effort_status)
	// 	{
	// 		if (effort_as_current)
	// 		{
	// 			double current;
	// 			interface_ptr_->get_current(motor, current);
	// 			temp.effort.push_back(current);
	// 		}
	// 		else
	// 		{
	// 			double torque;
	// 			interface_ptr_->get_torque(motor, torque);
	// 			temp.effort.push_back(torque);
	// 		}
	// 	}
	// 	else
	// 	{
	// 		temp.effort.push_back(-1);
	// 	}
	// }

	// this->motor_state_ = temp;
	auto message = std_msgs::msg::String();
	message.data = "Hai";
	this->motor_state_publisher_->publish(message);//this->motor_state_);
}

///
///
///
void MotorInterface::fault_callback()
{
	sensor_msgs::msg::JointState temp;

	interface_ptr_->clear_faults(params_.motor_names);

	drive_motors(motor_commands_);
}

///
///
///
void MotorInterface::drive_motors(sensor_msgs::msg::JointState &_msg)
{
	sensor_msgs::msg::JointState temp;

	for (int i = 0; i < _msg.name.size(); ++i)
	{
		temp.name[0] = _msg.name[0];
		temp.position[0] = _msg.position[0];
		temp.velocity[0] = _msg.velocity[0];
		temp.effort[0] = _msg.effort[0];
		drive_motor(temp);
	}
}

///
///
///
void MotorInterface::drive_motor(sensor_msgs::msg::JointState &_msg)
{
	if (_msg.position[0] != -1)
	{
		interface_ptr_->go_to_position_profile(_msg.name[0], _msg.position[0]);
	}
	else if (_msg.velocity[0] != -1)
	{
		interface_ptr_->go_to_velocity_profile(_msg.name[0], _msg.velocity[0]);
	}
	else if (_msg.effort[0] != -1)
	{
		interface_ptr_->go_to_torque(_msg.name[0], _msg.effort[0]);
	}
	else
	{
		RCLCPP_WARN(this->get_logger(), "No motor drive command");
	}
}

///
///
///
void MotorInterface::declare_params()
{
	// DECLARE PARAMS --------------------------------------------------

	this->declare_parameter("motor_names",std::vector<std::string>());
	//motor_names_param_ =rclcpp::Parameter("motor_names", std::vector<std::string>{"none"});
	//special_params_.push_back(motor_names_param);

	this->declare_parameter("motor_ids", std::vector<int64_t>());
	//rclcpp::Parameter motor_ids_param("motor/ids", std::vector<int>{0});
	//special_params_.push_back(motor_ids_param);

	this->declare_parameter("motor_params/gear_ratio", 1.0);
	this->declare_parameter("motor_params/counts_per_rev", 1);
	this->declare_parameter("motor_params/wheel_radius/cm", 1.0);
	this->declare_parameter("motor_params/kT", 1.0);
	this->declare_parameter("motor_params/user_limits/ang_vel_rpm", 1.0);
	this->declare_parameter("motor_params/user_limits/acc_rpm", 1.0);
	this->declare_parameter("motor_params/absolute_limits/curr_stall_a", 1.0);
	this->declare_parameter("motor_params/user_limits/curr_inst_a", 1.0);
	this->declare_parameter("motor_params/user_limits/curr_cont_a", 1.0);

	// EPOS Modules
	
	this->declare_parameter("epos_module/protocol", "MAXON SERIAL V2");
	this->declare_parameter("epos_module/com_interface", "USB");
	this->declare_parameter("epos_module/port", "USB0");
	this->declare_parameter("epos_module/baud_rate", 1000000);
	this->declare_parameter("motor_close_timeout", 30.0);

	// // Logging
	this->declare_parameter("logging/throttle", 1000);
	
}

epos2::EPOSParams MotorInterface::get_params()
{
	epos2::EPOSParams params;
	int special_param_counter = 0;

	// Motors
	this->get_parameter("motor_names", motor_names_param_);
	params.motor_names = motor_names_param_.as_string_array();
	//++special_param_counter;

	std::vector<int> ids;
	rclcpp::Parameter motor_ids_param = this->get_parameter("motor_ids");
	ids = std::vector<int>(motor_ids_param.as_integer_array().begin(),
						   motor_ids_param.as_integer_array().end());
	//++special_param_counter;

	for (std::vector<int>::size_type i = 0; i < ids.size(); ++i)
	{
		params.motor_ids.insert(std::make_pair(params.motor_names[i], ids[i]));
		params.motor_inds.insert(std::make_pair(params.motor_names[i], i));
	}

	// Maxon Motors
	epos2::MaxonMotor temp_motor;
	for (std::vector<int>::size_type i = 0; i < ids.size(); ++i)
	{
		temp_motor.index = ids[i];
		this->get_parameter("motor_params/gear_ratio", temp_motor.gear_ratio);
		this->get_parameter("motor_params/counts_per_rev", temp_motor.counts_per_rev);
		this->get_parameter("motor_params/wheel_radius/cm", temp_motor.wheel_radius);
		this->get_parameter("motor_params/kT", temp_motor.kT);
		this->get_parameter("motor_params/user_limits/ang_vel_rpm", temp_motor.ang_vel_limit);
		this->get_parameter("motor_params/user_limits/acc_rpm", temp_motor.acc_limit);
		this->get_parameter("motor_params/absolute_limits/curr_stall_a", temp_motor.stall_current);
		this->get_parameter("motor_params/user_limits/curr_inst_a", temp_motor.instantaneous_current_limit);
		this->get_parameter("motor_params/user_limits/curr_cont_a", temp_motor.continuous_current_limit);

		params.motors.push_back(temp_motor);
	}

	// EPOS Modules
	this->get_parameter("epos_module/protocol", params.protocol_stack_name);
	this->get_parameter("epos_module/com_interface", params.interface_name);
	this->get_parameter("epos_module/port", params.port_name);
	this->get_parameter("epos_module/baud_rate", params.baud_rate);
	this->get_parameter("motor_close_timeout", params.motor_close_timeout);

	// // Logging
	this->get_parameter("logging/throttle", params.throttle);

	return params;
}

///
///
///
MotorInterface::MotorInterface(std::string _node_name) : Node(_node_name, "ftr")
{
	// PARAM INITILIZATION ------------------------------------------------------------------------
	declare_params();
	this->params_ = get_params();

	//Publishers
	this->motor_state_publisher_ = this->create_publisher<std_msgs::msg::String>("/epos_motor_state", 10);
	// this->motor_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/epos_motor_state", 10);

	//Subscriptions
	motor_command_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
		"/epos_motor_command", 1, std::bind(&MotorInterface::motor_callback, this, _1));

	// Spin information
	int rate1, rate2;
	this->declare_parameter("/scope/status_timer", 1000);
	this->get_parameter("/scope/status_timer", rate1);
	this->declare_parameter("fault_time", 1000);
	this->get_parameter("fault_time", rate2);
	this->status_timer_ = this->create_wall_timer(std::chrono::milliseconds(rate1), std::bind(&MotorInterface::status_callback, this));
	this->fault_timer_ = this->create_wall_timer(std::chrono::milliseconds(rate2), std::bind(&MotorInterface::fault_callback, this));

	this->declare_parameter("position_status", false);
	this->get_parameter("position_status", position_status);
	this->declare_parameter("velocity_status", false);
	this->get_parameter("velocity_status", velocity_status);
	this->declare_parameter("effort_status", false);
	this->get_parameter("effort_status", effort_status);
	this->declare_parameter("effort_as_current", false);
	this->get_parameter("effort_as_current", effort_as_current);

	this->declare_parameter("epos_module/device", "EPOS4");
	this->get_parameter("epos_module/device", params_.device_name);

	interface_ptr_ = new epos2::EPOSWrapper(this, params_);

	

}

///
///
///
MotorInterface::~MotorInterface()
{
	interface_ptr_->~EPOSWrapper();
}