// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from epos_ros2_msgs:msg/MotorCommands.idl
// generated code does not contain a copyright notice
#include "epos_ros2_msgs/msg/detail/motor_commands__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `commands`
#include "epos_ros2_msgs/msg/detail/motor_command__functions.h"

bool
epos_ros2_msgs__msg__MotorCommands__init(epos_ros2_msgs__msg__MotorCommands * msg)
{
  if (!msg) {
    return false;
  }
  // commands
  if (!epos_ros2_msgs__msg__MotorCommand__Sequence__init(&msg->commands, 0)) {
    epos_ros2_msgs__msg__MotorCommands__fini(msg);
    return false;
  }
  return true;
}

void
epos_ros2_msgs__msg__MotorCommands__fini(epos_ros2_msgs__msg__MotorCommands * msg)
{
  if (!msg) {
    return;
  }
  // commands
  epos_ros2_msgs__msg__MotorCommand__Sequence__fini(&msg->commands);
}

epos_ros2_msgs__msg__MotorCommands *
epos_ros2_msgs__msg__MotorCommands__create()
{
  epos_ros2_msgs__msg__MotorCommands * msg = (epos_ros2_msgs__msg__MotorCommands *)malloc(sizeof(epos_ros2_msgs__msg__MotorCommands));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(epos_ros2_msgs__msg__MotorCommands));
  bool success = epos_ros2_msgs__msg__MotorCommands__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
epos_ros2_msgs__msg__MotorCommands__destroy(epos_ros2_msgs__msg__MotorCommands * msg)
{
  if (msg) {
    epos_ros2_msgs__msg__MotorCommands__fini(msg);
  }
  free(msg);
}


bool
epos_ros2_msgs__msg__MotorCommands__Sequence__init(epos_ros2_msgs__msg__MotorCommands__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  epos_ros2_msgs__msg__MotorCommands * data = NULL;
  if (size) {
    data = (epos_ros2_msgs__msg__MotorCommands *)calloc(size, sizeof(epos_ros2_msgs__msg__MotorCommands));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = epos_ros2_msgs__msg__MotorCommands__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        epos_ros2_msgs__msg__MotorCommands__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
epos_ros2_msgs__msg__MotorCommands__Sequence__fini(epos_ros2_msgs__msg__MotorCommands__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      epos_ros2_msgs__msg__MotorCommands__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

epos_ros2_msgs__msg__MotorCommands__Sequence *
epos_ros2_msgs__msg__MotorCommands__Sequence__create(size_t size)
{
  epos_ros2_msgs__msg__MotorCommands__Sequence * array = (epos_ros2_msgs__msg__MotorCommands__Sequence *)malloc(sizeof(epos_ros2_msgs__msg__MotorCommands__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = epos_ros2_msgs__msg__MotorCommands__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
epos_ros2_msgs__msg__MotorCommands__Sequence__destroy(epos_ros2_msgs__msg__MotorCommands__Sequence * array)
{
  if (array) {
    epos_ros2_msgs__msg__MotorCommands__Sequence__fini(array);
  }
  free(array);
}
