# CMake generated Testfile for 
# Source directory: /media/jared/32GB3/home/epos_ws/src/epos_ros2/epos_ros2_msgs
# Build directory: /media/jared/32GB3/home/epos_ws/src/epos_ros2/build/epos_ros2_msgs
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(copyright "/usr/bin/python3" "-u" "/opt/ros/galactic/share/ament_cmake_test/cmake/run_test.py" "/media/jared/32GB3/home/epos_ws/src/epos_ros2/build/epos_ros2_msgs/test_results/epos_ros2_msgs/copyright.xunit.xml" "--package-name" "epos_ros2_msgs" "--output-file" "/media/jared/32GB3/home/epos_ws/src/epos_ros2/build/epos_ros2_msgs/ament_copyright/copyright.txt" "--command" "/opt/ros/galactic/bin/ament_copyright" "--xunit-file" "/media/jared/32GB3/home/epos_ws/src/epos_ros2/build/epos_ros2_msgs/test_results/epos_ros2_msgs/copyright.xunit.xml")
set_tests_properties(copyright PROPERTIES  LABELS "copyright;linter" TIMEOUT "120" WORKING_DIRECTORY "/media/jared/32GB3/home/epos_ws/src/epos_ros2/epos_ros2_msgs" _BACKTRACE_TRIPLES "/opt/ros/galactic/share/ament_cmake_test/cmake/ament_add_test.cmake;124;add_test;/opt/ros/galactic/share/ament_cmake_copyright/cmake/ament_copyright.cmake;46;ament_add_test;/opt/ros/galactic/share/ament_cmake_copyright/cmake/ament_cmake_copyright_lint_hook.cmake;18;ament_copyright;/opt/ros/galactic/share/ament_cmake_copyright/cmake/ament_cmake_copyright_lint_hook.cmake;0;;/opt/ros/galactic/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/galactic/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/galactic/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/galactic/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/galactic/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/media/jared/32GB3/home/epos_ws/src/epos_ros2/epos_ros2_msgs/CMakeLists.txt;44;ament_package;/media/jared/32GB3/home/epos_ws/src/epos_ros2/epos_ros2_msgs/CMakeLists.txt;0;")
add_test(lint_cmake "/usr/bin/python3" "-u" "/opt/ros/galactic/share/ament_cmake_test/cmake/run_test.py" "/media/jared/32GB3/home/epos_ws/src/epos_ros2/build/epos_ros2_msgs/test_results/epos_ros2_msgs/lint_cmake.xunit.xml" "--package-name" "epos_ros2_msgs" "--output-file" "/media/jared/32GB3/home/epos_ws/src/epos_ros2/build/epos_ros2_msgs/ament_lint_cmake/lint_cmake.txt" "--command" "/opt/ros/galactic/bin/ament_lint_cmake" "--xunit-file" "/media/jared/32GB3/home/epos_ws/src/epos_ros2/build/epos_ros2_msgs/test_results/epos_ros2_msgs/lint_cmake.xunit.xml")
set_tests_properties(lint_cmake PROPERTIES  LABELS "lint_cmake;linter" TIMEOUT "60" WORKING_DIRECTORY "/media/jared/32GB3/home/epos_ws/src/epos_ros2/epos_ros2_msgs" _BACKTRACE_TRIPLES "/opt/ros/galactic/share/ament_cmake_test/cmake/ament_add_test.cmake;124;add_test;/opt/ros/galactic/share/ament_cmake_lint_cmake/cmake/ament_lint_cmake.cmake;47;ament_add_test;/opt/ros/galactic/share/ament_cmake_lint_cmake/cmake/ament_cmake_lint_cmake_lint_hook.cmake;21;ament_lint_cmake;/opt/ros/galactic/share/ament_cmake_lint_cmake/cmake/ament_cmake_lint_cmake_lint_hook.cmake;0;;/opt/ros/galactic/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/galactic/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/galactic/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/galactic/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/galactic/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/media/jared/32GB3/home/epos_ws/src/epos_ros2/epos_ros2_msgs/CMakeLists.txt;44;ament_package;/media/jared/32GB3/home/epos_ws/src/epos_ros2/epos_ros2_msgs/CMakeLists.txt;0;")
add_test(xmllint "/usr/bin/python3" "-u" "/opt/ros/galactic/share/ament_cmake_test/cmake/run_test.py" "/media/jared/32GB3/home/epos_ws/src/epos_ros2/build/epos_ros2_msgs/test_results/epos_ros2_msgs/xmllint.xunit.xml" "--package-name" "epos_ros2_msgs" "--output-file" "/media/jared/32GB3/home/epos_ws/src/epos_ros2/build/epos_ros2_msgs/ament_xmllint/xmllint.txt" "--command" "/opt/ros/galactic/bin/ament_xmllint" "--xunit-file" "/media/jared/32GB3/home/epos_ws/src/epos_ros2/build/epos_ros2_msgs/test_results/epos_ros2_msgs/xmllint.xunit.xml")
set_tests_properties(xmllint PROPERTIES  LABELS "xmllint;linter" TIMEOUT "60" WORKING_DIRECTORY "/media/jared/32GB3/home/epos_ws/src/epos_ros2/epos_ros2_msgs" _BACKTRACE_TRIPLES "/opt/ros/galactic/share/ament_cmake_test/cmake/ament_add_test.cmake;124;add_test;/opt/ros/galactic/share/ament_cmake_xmllint/cmake/ament_xmllint.cmake;50;ament_add_test;/opt/ros/galactic/share/ament_cmake_xmllint/cmake/ament_cmake_xmllint_lint_hook.cmake;18;ament_xmllint;/opt/ros/galactic/share/ament_cmake_xmllint/cmake/ament_cmake_xmllint_lint_hook.cmake;0;;/opt/ros/galactic/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/galactic/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/galactic/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/galactic/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/galactic/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/media/jared/32GB3/home/epos_ws/src/epos_ros2/epos_ros2_msgs/CMakeLists.txt;44;ament_package;/media/jared/32GB3/home/epos_ws/src/epos_ros2/epos_ros2_msgs/CMakeLists.txt;0;")
subdirs("epos_ros2_msgs__py")
