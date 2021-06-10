cmake_minimum_required(VERSION 3.5)
project(epos_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# EPOS Package from Maxon
set(EPOS_CMD_LIB_VER "6.6.1.0")

#find_package(ament_cmake REQUIRED COMPONENTS

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(epos_ros2_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Generate custom messages
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/MotorCommand.msg"
#   "msg/MotorCommands.msg"
#   DEPENDENCIES builtin_interfaces std_msgs
# )


#add_library(epos_cmd
#src/epos_cmd/epos_command.cpp
#)

set(ENV{EPOS_LIB_DIR} /opt/EposCmdLib_${EPOS_CMD_LIB_VER}/lib/x86_64)
link_directories($ENV{EPOS_LIB_DIR})
include_directories(include /opt/EposCmdLib_${EPOS_CMD_LIB_VER}/include)


add_executable(epos_fsm_node $ENV{EPOS_LIB_DIR} src/epos_cmd/epos_FSM_node.cpp)
target_link_libraries(epos_fsm_node 
  EposCmd 
  ftd2xx
  ${rclcpp_LIBRARIES}
)
#add_dependencies(epos_fsm_node ${PROJECT_NAME})
ament_target_dependencies(epos_fsm_node 
  rclcpp 
  std_msgs 
  epos_ros2_msgs
)
#rosidl_target_interfaces(epos_fsm_node
#     ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(epos_parameters src/utils/epos_param_server.cpp)
target_link_libraries(epos_parameters ${rclcpp_LIBRARIES})
#add_dependencies(epos_parameters ${PROJECT_NAME})
ament_target_dependencies(epos_parameters 
  rclcpp 
)

#add_dependencies(epos_fsm_node epos_ros_generate_messages_cpp)
#target_link_libraries(epos_fsm_node
#  ${catkin_LIBRARIES}
  #epos_cmd
#)

install (TARGETS
  epos_fsm_node
  DESTINATION lib/${PROJECT_NAME}
)
install (TARGETS
  epos_parameters
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

