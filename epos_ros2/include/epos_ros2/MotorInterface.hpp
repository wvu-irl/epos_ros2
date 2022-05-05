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

#include <sensor_msgs/msg/joint_state.hpp>

#include <epos_ros2/EPOSWrapper.hpp>
#include <epos_ros2/EPOSParams.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

class MotorInterface : public rclcpp::Node
{
public:
	epos2::EPOSWrapper *interface_ptr_;
	sensor_msgs::msg::JointState motor_commands_, motor_state_;

	MotorInterface(std::string _node_name);
	~MotorInterface();

private:
	epos2::EPOSParams params_;

	rclcpp::TimerBase::SharedPtr status_timer_, fault_timer_;
	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr motor_state_publisher_;
	// rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr error_monitor_publisher_;
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr motor_command_subscription_;

	void motor_callback(const sensor_msgs::msg::JointState::SharedPtr _msg);
	bool position_status = false, velocity_status = false, effort_status = false;
	bool effort_as_current = false;
	void status_callback();
	void fault_callback();

	void drive_motors(sensor_msgs::msg::JointState &_msg);
	void drive_motor(sensor_msgs::msg::JointState &_msg);

	std::vector<rclcpp::Parameter> special_params_;
	rclcpp::Parameter motor_names_param_;
	void declare_params();
	epos2::EPOSParams get_params();
};

#endif
