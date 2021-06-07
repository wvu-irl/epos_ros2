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
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(rclcpp REQUIRED)
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
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorCommand.msg"
  "msg/MotorCommands.msg"
  DEPENDENCIES 
 )


#add_library(epos_cmd
#src/epos_cmd/epos_command.cpp
#)

set(ENV{EPOS_LIB_DIR} /opt/EposCmdLib_${EPOS_CMD_LIB_VER}/lib/x86_64)
link_directories($ENV{EPOS_LIB_DIR})
include_directories(include /opt/EposCmdLib_${EPOS_CMD_LIB_VER}/include)

add_executable(epos_fsm_node $ENV{EPOS_LIB_DIR} src/epos_cmd/epos_FSM_node.cpp)
ament_target_dependencies(epos_fsm_node rclcpp std_msgs)
#add_dependencies(epos_fsm_node epos_ros_generate_messages_cpp)
#target_link_libraries(epos_fsm_node
#  ${catkin_LIBRARIES}
  #epos_cmd
#)

install (TARGETS
  epos_fsm_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

# add_executable(ft_test_control_node $ENV{EPOS_LIB_DIR} src/direct_control/ft_test_control.cpp)
# add_dependencies(ft_test_control_node epos_ros_generate_messages_cpp)
# target_link_libraries(ft_test_control_node
#   ${catkin_LIBRARIES}
#   epos_cmd
# )

# add_executable(hc_test_node $ENV{EPOS_LIB_DIR} src/epos_cmd/epos_hardcode_test_node.cpp)
# add_dependencies(hc_test_node epos_ros_generate_messages_cpp)
# target_link_libraries(hc_test_node
#   ${catkin_LIBRARIES}
#   epos_cmd
# )

# add_executable(gogoRover $ENV{EPOS_LIB_DIR} src/roverKinematics/URC_drive_conversion_node.cpp)
# add_dependencies(gogoRover epos_ros_generate_messages_cpp)
# target_link_libraries(gogoRover
#   ${catkin_LIBRARIES}
#   epos_cmd
# )

#add_executable(motor_node $ENV{EPOS_LIB_DIR} src/motor_cmd_node.cpp)
## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against


target_link_libraries(epos_fsm_node ${catkin_LIBRARIES} EposCmd ftd2xx)
# target_link_libraries(gogoRover ${catkin_LIBRARIES})
# target_link_libraries(test_node ${catkin_LIBRARIES} EposCmd ftd2xx)
# target_link_libraries(hc_test_node ${catkin_LIBRARIES} EposCmd ftd2xx)

# epos_cmd)
#target_link_libraries(motor_node ${catkin_LIBRARIES} EposCmd ftd2xx motor_cmd)
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_epos.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
