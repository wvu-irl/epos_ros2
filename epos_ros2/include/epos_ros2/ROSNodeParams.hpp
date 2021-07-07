/*
   Maxon Motor Controller EPOSWrappers ROS Parameters
   ROSParams.hpp
   Purpose: Link together various parameters for the EPOSWrapper

   @author Jared Beard
   @version 1.0 11/13/18
 */

#ifndef ROS_NODE_PARAMS_HPP
#define ROS_NODE_PARAMS_HPP

#include <rclcpp/rclcpp.hpp>

namespace epos2
{

  class ROSNodeParams
  {

  public:
    shared_ptr<rclcpp::Node> node_;

    ROSNodeParams(shared_ptr<rclcpp::Node> _node) : node_(_node);

    bool logging_condition()
    {
    }

  private:
  };
}
#endif