// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from epos_ros2_msgs:msg/MotorCommands.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "epos_ros2_msgs/msg/detail/motor_commands__rosidl_typesupport_introspection_c.h"
#include "epos_ros2_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "epos_ros2_msgs/msg/detail/motor_commands__functions.h"
#include "epos_ros2_msgs/msg/detail/motor_commands__struct.h"


// Include directives for member types
// Member `commands`
#include "epos_ros2_msgs/msg/motor_command.h"
// Member `commands`
#include "epos_ros2_msgs/msg/detail/motor_command__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void MotorCommands__rosidl_typesupport_introspection_c__MotorCommands_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  epos_ros2_msgs__msg__MotorCommands__init(message_memory);
}

void MotorCommands__rosidl_typesupport_introspection_c__MotorCommands_fini_function(void * message_memory)
{
  epos_ros2_msgs__msg__MotorCommands__fini(message_memory);
}

size_t MotorCommands__rosidl_typesupport_introspection_c__size_function__MotorCommand__commands(
  const void * untyped_member)
{
  const epos_ros2_msgs__msg__MotorCommand__Sequence * member =
    (const epos_ros2_msgs__msg__MotorCommand__Sequence *)(untyped_member);
  return member->size;
}

const void * MotorCommands__rosidl_typesupport_introspection_c__get_const_function__MotorCommand__commands(
  const void * untyped_member, size_t index)
{
  const epos_ros2_msgs__msg__MotorCommand__Sequence * member =
    (const epos_ros2_msgs__msg__MotorCommand__Sequence *)(untyped_member);
  return &member->data[index];
}

void * MotorCommands__rosidl_typesupport_introspection_c__get_function__MotorCommand__commands(
  void * untyped_member, size_t index)
{
  epos_ros2_msgs__msg__MotorCommand__Sequence * member =
    (epos_ros2_msgs__msg__MotorCommand__Sequence *)(untyped_member);
  return &member->data[index];
}

bool MotorCommands__rosidl_typesupport_introspection_c__resize_function__MotorCommand__commands(
  void * untyped_member, size_t size)
{
  epos_ros2_msgs__msg__MotorCommand__Sequence * member =
    (epos_ros2_msgs__msg__MotorCommand__Sequence *)(untyped_member);
  epos_ros2_msgs__msg__MotorCommand__Sequence__fini(member);
  return epos_ros2_msgs__msg__MotorCommand__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember MotorCommands__rosidl_typesupport_introspection_c__MotorCommands_message_member_array[1] = {
  {
    "commands",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(epos_ros2_msgs__msg__MotorCommands, commands),  // bytes offset in struct
    NULL,  // default value
    MotorCommands__rosidl_typesupport_introspection_c__size_function__MotorCommand__commands,  // size() function pointer
    MotorCommands__rosidl_typesupport_introspection_c__get_const_function__MotorCommand__commands,  // get_const(index) function pointer
    MotorCommands__rosidl_typesupport_introspection_c__get_function__MotorCommand__commands,  // get(index) function pointer
    MotorCommands__rosidl_typesupport_introspection_c__resize_function__MotorCommand__commands  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MotorCommands__rosidl_typesupport_introspection_c__MotorCommands_message_members = {
  "epos_ros2_msgs__msg",  // message namespace
  "MotorCommands",  // message name
  1,  // number of fields
  sizeof(epos_ros2_msgs__msg__MotorCommands),
  MotorCommands__rosidl_typesupport_introspection_c__MotorCommands_message_member_array,  // message members
  MotorCommands__rosidl_typesupport_introspection_c__MotorCommands_init_function,  // function to initialize message memory (memory has to be allocated)
  MotorCommands__rosidl_typesupport_introspection_c__MotorCommands_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MotorCommands__rosidl_typesupport_introspection_c__MotorCommands_message_type_support_handle = {
  0,
  &MotorCommands__rosidl_typesupport_introspection_c__MotorCommands_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_epos_ros2_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, epos_ros2_msgs, msg, MotorCommands)() {
  MotorCommands__rosidl_typesupport_introspection_c__MotorCommands_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, epos_ros2_msgs, msg, MotorCommand)();
  if (!MotorCommands__rosidl_typesupport_introspection_c__MotorCommands_message_type_support_handle.typesupport_identifier) {
    MotorCommands__rosidl_typesupport_introspection_c__MotorCommands_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MotorCommands__rosidl_typesupport_introspection_c__MotorCommands_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
