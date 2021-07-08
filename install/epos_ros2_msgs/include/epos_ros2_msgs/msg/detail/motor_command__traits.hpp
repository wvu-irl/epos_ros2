// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from epos_ros2_msgs:msg/MotorCommand.idl
// generated code does not contain a copyright notice

#ifndef EPOS_ROS2_MSGS__MSG__DETAIL__MOTOR_COMMAND__TRAITS_HPP_
#define EPOS_ROS2_MSGS__MSG__DETAIL__MOTOR_COMMAND__TRAITS_HPP_

#include "epos_ros2_msgs/msg/detail/motor_command__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const epos_ros2_msgs::msg::MotorCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: drive_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "drive_mode: ";
    value_to_yaml(msg.drive_mode, out);
    out << "\n";
  }

  // member: motor_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "motor_id: ";
    value_to_yaml(msg.motor_id, out);
    out << "\n";
  }

  // member: command
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "command: ";
    value_to_yaml(msg.command, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const epos_ros2_msgs::msg::MotorCommand & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<epos_ros2_msgs::msg::MotorCommand>()
{
  return "epos_ros2_msgs::msg::MotorCommand";
}

template<>
inline const char * name<epos_ros2_msgs::msg::MotorCommand>()
{
  return "epos_ros2_msgs/msg/MotorCommand";
}

template<>
struct has_fixed_size<epos_ros2_msgs::msg::MotorCommand>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<epos_ros2_msgs::msg::MotorCommand>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<epos_ros2_msgs::msg::MotorCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // EPOS_ROS2_MSGS__MSG__DETAIL__MOTOR_COMMAND__TRAITS_HPP_
