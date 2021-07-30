/**
 * @file utils.cpp
 * @author  Jared J Beard <jbeard6@mix.wvu.edu>
 * @version 1.0
 *
 * @section DESCRIPTION
 *
 * Utility functions for EPOSWrapper
 */
#include <epos_ros2/EPOSWrapper.hpp>

namespace epos2
{
    /////////////////////////////////////////////////////////////////////
    /**************************UTILS************************************/
    /////////////////////////////////////////////////////////////////////

    /// Includes gear ratio in output so that output is not that of motor
    /// but that of entire gear train
    ///
    double EPOSWrapper::rpm_2_mps(MaxonMotor &_motor, double _velocity)
    {
        RCLCPP_DEBUG(node_ptr_->get_logger(), "RPM to m/s");
        // 200 = 100 is for cm to m * 2 is for radius to diameter
        double circumference = _motor.wheel_radius*200.0*M_PI;
        // 60 is for min to s
        double v = (_velocity*circumference/60.0)/_motor.gear_ratio;
        std::string msg;
        msg = "Velocity converted from " + std::to_string(_velocity) + " RPM to " + std::to_string(v) + " m/s";
        RCLCPP_DEBUG(node_ptr_->get_logger(), msg.c_str());
        return v;
    }

    /// Includes gear ratio in output so that output is not that of motor
    /// but that of entire gear train
    ///
    double EPOSWrapper::mps_2_rpm(MaxonMotor &_motor, double _velocity)
    {
        RCLCPP_DEBUG(node_ptr_->get_logger(), "m/s to RPM");
        // 100 is for cm to m
        // 2 is for radius to diameter
        double circumference = _motor.wheel_radius*200.0*M_PI;
        // 60 is for min to s
        double v = (_velocity*_motor.gear_ratio*60.0)/circumference;
        std::string msg;
        msg = "Velocity converted from " + std::to_string(_velocity) + " m/s to " + std::to_string(v) + " RPM";
        RCLCPP_DEBUG(node_ptr_->get_logger(), msg.c_str());
        return v;
    }
}
