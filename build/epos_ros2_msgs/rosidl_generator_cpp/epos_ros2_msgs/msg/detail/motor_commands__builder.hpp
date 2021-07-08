// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from epos_ros2_msgs:msg/MotorCommands.idl
// generated code does not contain a copyright notice

#ifndef EPOS_ROS2_MSGS__MSG__DETAIL__MOTOR_COMMANDS__BUILDER_HPP_
#define EPOS_ROS2_MSGS__MSG__DETAIL__MOTOR_COMMANDS__BUILDER_HPP_

#include "epos_ros2_msgs/msg/detail/motor_commands__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace epos_ros2_msgs
{

namespace msg
{

namespace builder
{

class Init_MotorCommands_commands
{
public:
  Init_MotorCommands_commands()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::epos_ros2_msgs::msg::MotorCommands commands(::epos_ros2_msgs::msg::MotorCommands::_commands_type arg)
  {
    msg_.commands = std::move(arg);
    return std::move(msg_);
  }

private:
  ::epos_ros2_msgs::msg::MotorCommands msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::epos_ros2_msgs::msg::MotorCommands>()
{
  return epos_ros2_msgs::msg::builder::Init_MotorCommands_commands();
}

}  // namespace epos_ros2_msgs

#endif  // EPOS_ROS2_MSGS__MSG__DETAIL__MOTOR_COMMANDS__BUILDER_HPP_
