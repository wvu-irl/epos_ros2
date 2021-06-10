/*
   Utilities to interface with ROS
   EPOSUtils.hpp
   Purpose: Performs utility for epos related interfaces

   @author Jared Beard
   @version 1.0 6/4/21
 */
#ifndef EPOS_UTILS_H
#define EPOS_UTILS_H

#include <string>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/int64_multi_array.hpp>

//#include <epos_ros/msg/epos_command.h>
#include <epos_ros2_msgs/msg/motor_command.hpp>
#include <epos_ros2_msgs/msg/motor_commands.hpp>


class EPOSUtils
{
	std::map<std::string, int> map_;
	
	public:
		EPOSUtils(std::vector<std::string> names, std::vector<int> ids);
		
  	private:
	  int get_motor_id(std::string name);
  	  std::vector<int> get_motor_ids(std::vector<std::string> name);

	  std::string get_motor_name(int id);
	  std::vector<std::string> get_motor_names(std::vector<int> ids);




		
    	

};

#endif