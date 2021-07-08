// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from epos_ros2_msgs:msg/MotorCommands.idl
// generated code does not contain a copyright notice

#ifndef EPOS_ROS2_MSGS__MSG__DETAIL__MOTOR_COMMANDS__STRUCT_HPP_
#define EPOS_ROS2_MSGS__MSG__DETAIL__MOTOR_COMMANDS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'commands'
#include "epos_ros2_msgs/msg/detail/motor_command__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__epos_ros2_msgs__msg__MotorCommands __attribute__((deprecated))
#else
# define DEPRECATED__epos_ros2_msgs__msg__MotorCommands __declspec(deprecated)
#endif

namespace epos_ros2_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MotorCommands_
{
  using Type = MotorCommands_<ContainerAllocator>;

  explicit MotorCommands_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit MotorCommands_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _commands_type =
    std::vector<epos_ros2_msgs::msg::MotorCommand_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<epos_ros2_msgs::msg::MotorCommand_<ContainerAllocator>>>;
  _commands_type commands;

  // setters for named parameter idiom
  Type & set__commands(
    const std::vector<epos_ros2_msgs::msg::MotorCommand_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<epos_ros2_msgs::msg::MotorCommand_<ContainerAllocator>>> & _arg)
  {
    this->commands = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    epos_ros2_msgs::msg::MotorCommands_<ContainerAllocator> *;
  using ConstRawPtr =
    const epos_ros2_msgs::msg::MotorCommands_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<epos_ros2_msgs::msg::MotorCommands_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<epos_ros2_msgs::msg::MotorCommands_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      epos_ros2_msgs::msg::MotorCommands_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<epos_ros2_msgs::msg::MotorCommands_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      epos_ros2_msgs::msg::MotorCommands_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<epos_ros2_msgs::msg::MotorCommands_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<epos_ros2_msgs::msg::MotorCommands_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<epos_ros2_msgs::msg::MotorCommands_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__epos_ros2_msgs__msg__MotorCommands
    std::shared_ptr<epos_ros2_msgs::msg::MotorCommands_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__epos_ros2_msgs__msg__MotorCommands
    std::shared_ptr<epos_ros2_msgs::msg::MotorCommands_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotorCommands_ & other) const
  {
    if (this->commands != other.commands) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotorCommands_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotorCommands_

// alias to use template instance with default allocator
using MotorCommands =
  epos_ros2_msgs::msg::MotorCommands_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace epos_ros2_msgs

#endif  // EPOS_ROS2_MSGS__MSG__DETAIL__MOTOR_COMMANDS__STRUCT_HPP_
