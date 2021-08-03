/*
   Maxon Motor Controller interface node
   epos_interface_node.cpp
   Purpose: Deployable EPOS code for most projects

   @author Jared Beard
   @version 1.0 6/4/21
 */

#include <stdio.h>
#include <cmath>
#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <epos_ros2/MotorInterface.hpp>

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MotorInterface>("epos_interface_node"));
	rclcpp::shutdown();
	return 0;
}
