// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from epos_ros2_msgs:msg/MotorCommand.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "epos_ros2_msgs/msg/detail/motor_command__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace epos_ros2_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void MotorCommand_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) epos_ros2_msgs::msg::MotorCommand(_init);
}

void MotorCommand_fini_function(void * message_memory)
{
  auto typed_message = static_cast<epos_ros2_msgs::msg::MotorCommand *>(message_memory);
  typed_message->~MotorCommand();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MotorCommand_message_member_array[3] = {
  {
    "drive_mode",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(epos_ros2_msgs::msg::MotorCommand, drive_mode),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "motor_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(epos_ros2_msgs::msg::MotorCommand, motor_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "command",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(epos_ros2_msgs::msg::MotorCommand, command),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MotorCommand_message_members = {
  "epos_ros2_msgs::msg",  // message namespace
  "MotorCommand",  // message name
  3,  // number of fields
  sizeof(epos_ros2_msgs::msg::MotorCommand),
  MotorCommand_message_member_array,  // message members
  MotorCommand_init_function,  // function to initialize message memory (memory has to be allocated)
  MotorCommand_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MotorCommand_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MotorCommand_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace epos_ros2_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<epos_ros2_msgs::msg::MotorCommand>()
{
  return &::epos_ros2_msgs::msg::rosidl_typesupport_introspection_cpp::MotorCommand_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, epos_ros2_msgs, msg, MotorCommand)() {
  return &::epos_ros2_msgs::msg::rosidl_typesupport_introspection_cpp::MotorCommand_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
