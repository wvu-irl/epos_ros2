// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from epos_ros2_msgs:msg/MotorCommands.idl
// generated code does not contain a copyright notice

#ifndef EPOS_ROS2_MSGS__MSG__DETAIL__MOTOR_COMMANDS__FUNCTIONS_H_
#define EPOS_ROS2_MSGS__MSG__DETAIL__MOTOR_COMMANDS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "epos_ros2_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "epos_ros2_msgs/msg/detail/motor_commands__struct.h"

/// Initialize msg/MotorCommands message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * epos_ros2_msgs__msg__MotorCommands
 * )) before or use
 * epos_ros2_msgs__msg__MotorCommands__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_epos_ros2_msgs
bool
epos_ros2_msgs__msg__MotorCommands__init(epos_ros2_msgs__msg__MotorCommands * msg);

/// Finalize msg/MotorCommands message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_epos_ros2_msgs
void
epos_ros2_msgs__msg__MotorCommands__fini(epos_ros2_msgs__msg__MotorCommands * msg);

/// Create msg/MotorCommands message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * epos_ros2_msgs__msg__MotorCommands__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_epos_ros2_msgs
epos_ros2_msgs__msg__MotorCommands *
epos_ros2_msgs__msg__MotorCommands__create();

/// Destroy msg/MotorCommands message.
/**
 * It calls
 * epos_ros2_msgs__msg__MotorCommands__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_epos_ros2_msgs
void
epos_ros2_msgs__msg__MotorCommands__destroy(epos_ros2_msgs__msg__MotorCommands * msg);


/// Initialize array of msg/MotorCommands messages.
/**
 * It allocates the memory for the number of elements and calls
 * epos_ros2_msgs__msg__MotorCommands__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_epos_ros2_msgs
bool
epos_ros2_msgs__msg__MotorCommands__Sequence__init(epos_ros2_msgs__msg__MotorCommands__Sequence * array, size_t size);

/// Finalize array of msg/MotorCommands messages.
/**
 * It calls
 * epos_ros2_msgs__msg__MotorCommands__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_epos_ros2_msgs
void
epos_ros2_msgs__msg__MotorCommands__Sequence__fini(epos_ros2_msgs__msg__MotorCommands__Sequence * array);

/// Create array of msg/MotorCommands messages.
/**
 * It allocates the memory for the array and calls
 * epos_ros2_msgs__msg__MotorCommands__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_epos_ros2_msgs
epos_ros2_msgs__msg__MotorCommands__Sequence *
epos_ros2_msgs__msg__MotorCommands__Sequence__create(size_t size);

/// Destroy array of msg/MotorCommands messages.
/**
 * It calls
 * epos_ros2_msgs__msg__MotorCommands__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_epos_ros2_msgs
void
epos_ros2_msgs__msg__MotorCommands__Sequence__destroy(epos_ros2_msgs__msg__MotorCommands__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // EPOS_ROS2_MSGS__MSG__DETAIL__MOTOR_COMMANDS__FUNCTIONS_H_
