// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from epos_ros2_msgs:msg/MotorCommand.idl
// generated code does not contain a copyright notice

#ifndef EPOS_ROS2_MSGS__MSG__DETAIL__MOTOR_COMMAND__STRUCT_HPP_
#define EPOS_ROS2_MSGS__MSG__DETAIL__MOTOR_COMMAND__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__epos_ros2_msgs__msg__MotorCommand __attribute__((deprecated))
#else
# define DEPRECATED__epos_ros2_msgs__msg__MotorCommand __declspec(deprecated)
#endif

namespace epos_ros2_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MotorCommand_
{
  using Type = MotorCommand_<ContainerAllocator>;

  explicit MotorCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->drive_mode = "";
      this->motor_id = 0;
      this->command = 0ll;
    }
  }

  explicit MotorCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : drive_mode(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->drive_mode = "";
      this->motor_id = 0;
      this->command = 0ll;
    }
  }

  // field types and members
  using _drive_mode_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _drive_mode_type drive_mode;
  using _motor_id_type =
    int16_t;
  _motor_id_type motor_id;
  using _command_type =
    int64_t;
  _command_type command;

  // setters for named parameter idiom
  Type & set__drive_mode(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->drive_mode = _arg;
    return *this;
  }
  Type & set__motor_id(
    const int16_t & _arg)
  {
    this->motor_id = _arg;
    return *this;
  }
  Type & set__command(
    const int64_t & _arg)
  {
    this->command = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    epos_ros2_msgs::msg::MotorCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const epos_ros2_msgs::msg::MotorCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<epos_ros2_msgs::msg::MotorCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<epos_ros2_msgs::msg::MotorCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      epos_ros2_msgs::msg::MotorCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<epos_ros2_msgs::msg::MotorCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      epos_ros2_msgs::msg::MotorCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<epos_ros2_msgs::msg::MotorCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<epos_ros2_msgs::msg::MotorCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<epos_ros2_msgs::msg::MotorCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__epos_ros2_msgs__msg__MotorCommand
    std::shared_ptr<epos_ros2_msgs::msg::MotorCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__epos_ros2_msgs__msg__MotorCommand
    std::shared_ptr<epos_ros2_msgs::msg::MotorCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotorCommand_ & other) const
  {
    if (this->drive_mode != other.drive_mode) {
      return false;
    }
    if (this->motor_id != other.motor_id) {
      return false;
    }
    if (this->command != other.command) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotorCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotorCommand_

// alias to use template instance with default allocator
using MotorCommand =
  epos_ros2_msgs::msg::MotorCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace epos_ros2_msgs

#endif  // EPOS_ROS2_MSGS__MSG__DETAIL__MOTOR_COMMAND__STRUCT_HPP_
