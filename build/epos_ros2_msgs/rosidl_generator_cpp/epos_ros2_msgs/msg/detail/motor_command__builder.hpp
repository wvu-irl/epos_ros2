// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from epos_ros2_msgs:msg/MotorCommand.idl
// generated code does not contain a copyright notice

#ifndef EPOS_ROS2_MSGS__MSG__DETAIL__MOTOR_COMMAND__BUILDER_HPP_
#define EPOS_ROS2_MSGS__MSG__DETAIL__MOTOR_COMMAND__BUILDER_HPP_

#include "epos_ros2_msgs/msg/detail/motor_command__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace epos_ros2_msgs
{

namespace msg
{

namespace builder
{

class Init_MotorCommand_command
{
public:
  explicit Init_MotorCommand_command(::epos_ros2_msgs::msg::MotorCommand & msg)
  : msg_(msg)
  {}
  ::epos_ros2_msgs::msg::MotorCommand command(::epos_ros2_msgs::msg::MotorCommand::_command_type arg)
  {
    msg_.command = std::move(arg);
    return std::move(msg_);
  }

private:
  ::epos_ros2_msgs::msg::MotorCommand msg_;
};

class Init_MotorCommand_motor_id
{
public:
  explicit Init_MotorCommand_motor_id(::epos_ros2_msgs::msg::MotorCommand & msg)
  : msg_(msg)
  {}
  Init_MotorCommand_command motor_id(::epos_ros2_msgs::msg::MotorCommand::_motor_id_type arg)
  {
    msg_.motor_id = std::move(arg);
    return Init_MotorCommand_command(msg_);
  }

private:
  ::epos_ros2_msgs::msg::MotorCommand msg_;
};

class Init_MotorCommand_drive_mode
{
public:
  Init_MotorCommand_drive_mode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorCommand_motor_id drive_mode(::epos_ros2_msgs::msg::MotorCommand::_drive_mode_type arg)
  {
    msg_.drive_mode = std::move(arg);
    return Init_MotorCommand_motor_id(msg_);
  }

private:
  ::epos_ros2_msgs::msg::MotorCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::epos_ros2_msgs::msg::MotorCommand>()
{
  return epos_ros2_msgs::msg::builder::Init_MotorCommand_drive_mode();
}

}  // namespace epos_ros2_msgs

#endif  // EPOS_ROS2_MSGS__MSG__DETAIL__MOTOR_COMMAND__BUILDER_HPP_
