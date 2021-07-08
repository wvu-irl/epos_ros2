// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from epos_ros2_msgs:msg/MotorCommands.idl
// generated code does not contain a copyright notice

#ifndef EPOS_ROS2_MSGS__MSG__DETAIL__MOTOR_COMMANDS__STRUCT_H_
#define EPOS_ROS2_MSGS__MSG__DETAIL__MOTOR_COMMANDS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'commands'
#include "epos_ros2_msgs/msg/detail/motor_command__struct.h"

// Struct defined in msg/MotorCommands in the package epos_ros2_msgs.
typedef struct epos_ros2_msgs__msg__MotorCommands
{
  epos_ros2_msgs__msg__MotorCommand__Sequence commands;
} epos_ros2_msgs__msg__MotorCommands;

// Struct for a sequence of epos_ros2_msgs__msg__MotorCommands.
typedef struct epos_ros2_msgs__msg__MotorCommands__Sequence
{
  epos_ros2_msgs__msg__MotorCommands * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} epos_ros2_msgs__msg__MotorCommands__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // EPOS_ROS2_MSGS__MSG__DETAIL__MOTOR_COMMANDS__STRUCT_H_
