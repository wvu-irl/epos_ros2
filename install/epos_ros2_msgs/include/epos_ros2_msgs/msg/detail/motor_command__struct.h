// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from epos_ros2_msgs:msg/MotorCommand.idl
// generated code does not contain a copyright notice

#ifndef EPOS_ROS2_MSGS__MSG__DETAIL__MOTOR_COMMAND__STRUCT_H_
#define EPOS_ROS2_MSGS__MSG__DETAIL__MOTOR_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'drive_mode'
#include "rosidl_runtime_c/string.h"

// Struct defined in msg/MotorCommand in the package epos_ros2_msgs.
typedef struct epos_ros2_msgs__msg__MotorCommand
{
  rosidl_runtime_c__String drive_mode;
  int16_t motor_id;
  int64_t command;
} epos_ros2_msgs__msg__MotorCommand;

// Struct for a sequence of epos_ros2_msgs__msg__MotorCommand.
typedef struct epos_ros2_msgs__msg__MotorCommand__Sequence
{
  epos_ros2_msgs__msg__MotorCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} epos_ros2_msgs__msg__MotorCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // EPOS_ROS2_MSGS__MSG__DETAIL__MOTOR_COMMAND__STRUCT_H_
