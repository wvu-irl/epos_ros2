#ifndef MOTOR_INTERFACE_H
#define MOTOR_INTERFACE_H

#include <rclcpp/rclcpp/hpp>

//#include <epos_ros/msg/epos_command.h>
#include <epos_ros/msg/MotorCommands.h>
#include <std_msgs/msg/Int64MultiArray.h>

class MotorInterface : public rclcpp::Node
{
	public:
		epos_ros::MotorCommands motor_commands_; 
		//modes 1 = velocity, 2 = torque, 3 = current, 4 = position, 5 = homing

		MotorSubscriber(std::string node_name) : Node (node_name)
		{
			//Publishers
			motor_position_publisher_ = this->create_publisher<std_msgs::msg::Int64MultiArray>("/motor_positions", 10);
      		
			//Subscriptions
      		motor_subscription_ = this->create_subscription<epos_ros::msg::MotorCommands>(
      			"/epos_command", 1, std::bind(&MinimalSubscriber::motor_callback, this, _1));

			// Spin information
			//timer_ = this->create_wall_timer(20ms, std::bind(&MotorInterface::motor_callback, this));
    	}
  	private:
		//rclcpp::TimerBase::SharedPtr timer_;
    	rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr motor_position_publisher_;
		rclcpp::Subscription<std_msgs::msg::MotorCommands>::SharedPtr motor_subscription_;

    	void motor_callback(const epos_ros::msg::MotorCommands::SharedPtr msg) const
    	{
			motor_commands_ = *msg;
			//RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    	}
    	

};

#endif