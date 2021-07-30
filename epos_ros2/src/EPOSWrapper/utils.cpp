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
        // 200 = 100 is for cm to m * 2 is for radius to diameter
        double circumference = _motor.wheel_radius*200.0*M_PI;
        // 60 is for min to s
        return (_velocity*circumference/60.0)/_motor.gear_ratio;
    }

    /// Includes gear ratio in output so that output is not that of motor
    /// but that of entire gear train
    ///
    double EPOSWrapper::mps_2_rpm(MaxonMotor &_motor, double _velocity)
    {
        // 100 is for cm to m
        // 2 is for radius to diameter
        double circumference = _motor.wheel_radius*200.0*M_PI;
        // 60 is for min to s
        return (_velocity*_motor.gear_ratio*60.0)/circumference;
    }
}
