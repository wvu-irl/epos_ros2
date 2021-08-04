cmake_minimum_required(VERSION 3.5)
project(epos_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
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
find_package(sensor_msgs REQUIRED)
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
set(ENV{EPOS_LIB_DIR} /opt/EposCmdLib_${EPOS_CMD_LIB_VER}/lib/x86_64)
link_directories($ENV{EPOS_LIB_DIR})
include_directories(include /opt/EposCmdLib_${EPOS_CMD_LIB_VER}/include)

add_library(epos_wrapper_lib
  ${PROJECT_SOURCE_DIR}/src/EPOSWrapper/configuration.cpp
  ${PROJECT_SOURCE_DIR}/src/EPOSWrapper/error_log.cpp
  ${PROJECT_SOURCE_DIR}/src/EPOSWrapper/initialization.cpp
  ${PROJECT_SOURCE_DIR}/src/EPOSWrapper/operation.cpp
  ${PROJECT_SOURCE_DIR}/src/EPOSWrapper/utils.cpp
)
set(DEPENDENCIES
  "rclcpp"
)
target_link_libraries(epos_wrapper_lib ftd2xx EposCmd)
ament_target_dependencies(epos_wrapper_lib ${DEPENDENCIES})

add_library(motor_interface_lib
src/MotorInterface.cpp
)
# set(DEPENDENCIES
#   "rclcpp"
# )
target_link_libraries(motor_interface_lib epos_wrapper_lib)
ament_target_dependencies(motor_interface_lib sensor_msgs rclcpp)


add_executable(epos_interface_node $ENV{EPOS_LIB_DIR} src/epos_interface_node.cpp)
target_link_libraries(epos_interface_node 
  epos_wrapper_lib 
  motor_interface_lib
  #ftd2xx
  ${rclcpp_LIBRARIES}
)
# add_dependencies(epos_interface_node ${PROJECT_NAME}/src/epos_interface_node.cpp)
ament_target_dependencies(epos_interface_node 
  rclcpp 
)
#rosidl_target_interfaces(epos_interface_node
#     ${PROJECT_NAME} "rosidl_typesupport_cpp")

# add_executable(epos_parameters src/utils/epos_param_server.cpp)
# target_link_libraries(epos_parameters ${rclcpp_LIBRARIES})
# #add_dependencies(epos_parameters ${PROJECT_NAME})
# ament_target_dependencies(epos_parameters 
#   rclcpp 
# )

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

  install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install (TARGETS
  epos_interface_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

