/*
   ROS Interface for EPOS nodes
   MotorInterface.hpp
   Purpose: Allow EPOS nodes to communicate with ROS

   @author Jared Beard
   @version 1.0 6/4/21
 */
// #ifndef MOTOR_INTERFACE_H
// #define MOTOR_INTERFACE_H

#include <epos_ros2/MotorInterface.hpp>


MotorInterface::MotorInterface(std::string _node_name) : Node(_node_name)
{
	// PARAM INITILIZATION ------------------------------------------------------------------------
	epos2::EPOSParams epos_params;
	//Publishers
	motor_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/epos_motor_state", 10);

	//Subscriptions
	motor_command_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
		"/epos_motor_command", 1, std::bind(&MotorInterface::motor_callback, this, _1));

	// Spin information
	main_ = this->create_wall_timer(20ms, std::bind(&MotorInterface::main_callback, this));
	interface_ptr_ = new epos2::EPOSWrapper(epos_params);
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