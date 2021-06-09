/*
   Utilities to interface with ROS
   EPOSUtils.hpp
   Purpose: Allow EPOS nodes to communicate with ROS

   @author Jared Beard
   @version 1.0 6/4/21
 */
#ifndef MOTOR_INTERFACE_H
#define MOTOR_INTERFACE_H

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/int64_multi_array.hpp>

//#include <epos_ros/msg/epos_command.h>
#include <epos_ros2_msgs/msg/motor_command.hpp>
#include <epos_ros2_msgs/msg/motor_commands.hpp>

using std::placeholders::_1;

class MotorInterface : public rclcpp::Node
{
	public:
		epos_ros2_msgs::msg::MotorCommands motor_commands_; 
		//modes 1 = velocity, 2 = torque, 3 = current, 4 = position, 5 = homing

		MotorInterface(std::string node_name) : Node (node_name)
		{
			//Publishers
			motor_position_publisher_ = this->create_publisher<std_msgs::msg::Int64MultiArray>("/motor_positions", 10);
      		
			//Subscriptions
      		motor_subscription_ = this->create_subscription<epos_ros2_msgs::msg::MotorCommands>(
      			"/epos_command", 1, std::bind(&MotorInterface::motor_callback, this, _1));

			// Spin information
			//timer_ = this->create_wall_timer(20ms, std::bind(&MotorInterface::motor_callback, this));
    	}
  	private:
		//rclcpp::TimerBase::SharedPtr timer_;
    	rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr motor_position_publisher_;
		rclcpp::Subscription<epos_ros2_msgs::msg::MotorCommands>::SharedPtr motor_subscription_;

    	void motor_callback(const epos_ros2_msgs::msg::MotorCommands::SharedPtr msg)
    	{
			motor_commands_ = *msg;
			//RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    	}
    	

};

#endif