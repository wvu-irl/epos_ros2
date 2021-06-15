/*
   ROS Interface for EPOS nodes
   MotorInterface.hpp
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

#include <epos_ros2/EPOSWrapper.hpp>

using std::placeholders::_1;

class MotorInterface : public rclcpp::Node
{
	public:
		EPOSWrapper interface;

		epos_ros2_msgs::msg::MotorCommands motor_commands_; 
		//modes 1 = velocity, 2 = torque, 3 = current, 4 = position, 5 = homing

		MotorInterface(std::string node_name) : Node (node_name);
		MotorInterface(std::string node_name,std::vector<int> _ids, int _br) : Node (node_name);

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