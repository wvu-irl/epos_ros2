// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from epos_ros2_msgs:msg/MotorCommands.idl
// generated code does not contain a copyright notice

#ifndef EPOS_ROS2_MSGS__MSG__DETAIL__MOTOR_COMMANDS__TRAITS_HPP_
#define EPOS_ROS2_MSGS__MSG__DETAIL__MOTOR_COMMANDS__TRAITS_HPP_

#include "epos_ros2_msgs/msg/detail/motor_commands__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

// Include directives for member types
// Member 'commands'
#include "epos_ros2_msgs/msg/detail/motor_command__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const epos_ros2_msgs::msg::MotorCommands & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: commands
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.commands.size() == 0) {
      out << "commands: []\n";
    } else {
      out << "commands:\n";
      for (auto item : msg.commands) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const epos_ros2_msgs::msg::MotorCommands & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<epos_ros2_msgs::msg::MotorCommands>()
{
  return "epos_ros2_msgs::msg::MotorCommands";
}

template<>
inline const char * name<epos_ros2_msgs::msg::MotorCommands>()
{
  return "epos_ros2_msgs/msg/MotorCommands";
}

template<>
struct has_fixed_size<epos_ros2_msgs::msg::MotorCommands>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<epos_ros2_msgs::msg::MotorCommands>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<epos_ros2_msgs::msg::MotorCommands>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // EPOS_ROS2_MSGS__MSG__DETAIL__MOTOR_COMMANDS__TRAITS_HPP_
