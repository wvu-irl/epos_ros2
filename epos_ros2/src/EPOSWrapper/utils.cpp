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
        double circumference = _motor.wheel_radius * 200.0 * M_PI;
        // 60 is for min to s
        double v = (_velocity * circumference / 60.0) / _motor.gear_ratio;
        std::string msg;
        msg = "Velocity converted from " + std::to_string(_velocity) + " RPM to " + std::to_string(v) + " m/s";
        RCLCPP_INFO_THROTTLE(node_ptr_->get_logger(), clock_, params_.throttle, msg.c_str());
        return v;
    }

    /// Includes gear ratio in output so that output is not that of motor
    /// but that of entire gear train
    ///
    double EPOSWrapper::mps_2_rpm(MaxonMotor &_motor, int _velocity)
    {
        RCLCPP_DEBUG(node_ptr_->get_logger(), "m/s to RPM");
        // 100 is for cm to m
        // 2 is for radius to diameter
        double circumference = _motor.wheel_radius * 200.0 * M_PI;
        // 60 is for min to s
        double v = (_velocity * _motor.gear_ratio * 60.0) / circumference;
        std::string msg;
        msg = "Velocity converted from " + std::to_string(_velocity) + " m/s to " + std::to_string(v) + " RPM";
        RCLCPP_INFO_THROTTLE(node_ptr_->get_logger(), clock_, params_.throttle, msg.c_str());
        return v;
    }

    /// Includes gear ratio in output so that output is not that of motor
    /// but that of entire gear train
    ///
    double EPOSWrapper::count_2_m(MaxonMotor &_motor, double _distance)
    {
        RCLCPP_DEBUG(node_ptr_->get_logger(), "encoder counts to m");
        // 100 is for cm to m
        // 2 is for radius to diameter
        double circumference = _motor.wheel_radius * 200.0 * M_PI;
        // 60 is for min to s
        double d = (_distance / _motor.counts_per_rev) * _motor.gear_ratio * circumference;
        std::string msg;
        msg = "Distance converted from " + std::to_string(_distance) + " counts to " + std::to_string(d) + " m";
        RCLCPP_INFO_THROTTLE(node_ptr_->get_logger(), clock_, params_.throttle, msg.c_str());
        return d;
    }

    /// Includes gear ratio in output so that output is not that of motor
    /// but that of entire gear train
    ///
    double EPOSWrapper::m_2_count(MaxonMotor &_motor, double _distance)
    {
        RCLCPP_DEBUG(node_ptr_->get_logger(), "m to encoder counts");
        // 100 is for cm to m
        // 2 is for radius to diameter
        double circumference = _motor.wheel_radius * 200.0 * M_PI;
        // 60 is for min to s
        double d = _distance / (_motor.gear_ratio * circumference) * _motor.counts_per_rev;
        std::string msg;
        msg = "Distance converted from " + std::to_string(_distance) + " m to " + std::to_string(d) + " counts";
        RCLCPP_INFO_THROTTLE(node_ptr_->get_logger(), clock_, params_.throttle, msg.c_str());
        return d;
    }

    ///
    ///
    ///
    double EPOSWrapper::mNm_2_ma(MaxonMotor &_motor, double _torque)
    {
        RCLCPP_DEBUG(node_ptr_->get_logger(), "mNm to mA");
        double current = _torque / (_motor.kT * _motor.gear_ratio);
        std::string msg;
        msg = "Torque converted from " + std::to_string(_torque) + " mNm to " + std::to_string(current) + " mA";
        RCLCPP_INFO_THROTTLE(node_ptr_->get_logger(), clock_, params_.throttle, msg.c_str());
        return current;
    }

    ///
    ///
    ///
    double EPOSWrapper::ma_2_mNm(MaxonMotor &_motor, double _current)
    {
        RCLCPP_DEBUG(node_ptr_->get_logger(), "mA to mNm");
        double torque = _current * _motor.kT * _motor.gear_ratio;
        std::string msg;
        msg = "Current converted from " + std::to_string(_current) + " mA to " + std::to_string(torque) + " mNm";
        RCLCPP_INFO_THROTTLE(node_ptr_->get_logger(), clock_, params_.throttle, msg.c_str());
        return torque;
    }
}
