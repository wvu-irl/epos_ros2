/*
   Maxon Motor Controller Finite State Machine node epos_test_node
   epos_FSM_node.cpp
   Purpose: Deployable EPOS code for most projects un epos commands and test behavior

   @author Jared Beard
   @version 1.0 6/4/21
 */


#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/int64_multi_array.hpp> // make a msg to wrap around this so we can specify motor

//#include <epos_ros/msg/EPOSCommand.h>
#include <epos_ros2_msgs/msg/motor_command.hpp>
#include <epos_ros2_msgs/msg/motor_commands.hpp>

#include <epos_ros2/MotorInterface.hpp>

#include <stdio.h>
#include <cmath>
#include <string>
#include <memory>




int main(int argc, char** argv)
{
		// ROS INITILIZATION ------------------------------------------------------------------------
		rclcpp::init(argc,argv);
		MotorInterface interface("epos_fsm_node");
				

		// IMPORT CONFIGURATION ---------------------------------------------------------------------
		// nh.param("motors_ids/list", motorIDs);
		// int baudrate;
		// nh.param("baudrate", baudrate, 1000000);

		// for (int i = 0; i < motorIDs.size(); ++i) cmd.push_back(0);

		// double gearRatio, countsPerRev;
		// nh.param("gear_ratio", gearRatio, 126.0);
		// nh.param("counts_per_rev", countsPerRev, 128.0);

		// double countsPreRevShaft = countsPerRev*gearRatio;

		// // EPOS CONFIGURATION/PREPARATION------------------------------------------------------------
		// EPOSCommand motorController(motorIDs, baudrate);
		// if (!motorController.openDevices())
		// {
		// 		ROS_FATAL("Motor not opened");
		// 		return 1;
		// }

		// motorController.enableMotors(motorIDs);
		// // FSM --------------------------------------------------------------------------------------
		// bool runMotors = true;
		// ros::Rate rate(20);
		// std::vector<int> positions;
		// while(rclcpp::ok() && runMotors)
		// {
		// 		if(motorController.enableMotors(motorIDs))
		// 		{
		// 				// VELOCITY -----------------------------------------------------------------------------
		// 				if (operationMode == "velocity")
		// 				{
		// 						motorController.goToVel(motorIDs, cmd);

		// 						// VELOCITY -----------------------------------------------------------------------------
		// 				} else if (operationMode == "torque")
		// 				{
		// 						motorController.goToTorque(motorIDs, cmd, gearRatio);

		// 						// CURRENT ------------------------------------------------------------------------------
		// 				} else if (operationMode == "current")
		// 				{

		// 						// POSITION -----------------------------------------------------------------------------
		// 				} else if (operationMode == "position")
		// 				{

		// 						// HOMING -------------------------------------------------------------------------------
		// 				} else if (operationMode == "homing")
		// 				{

		// 						// FAILED -------------------------------------------------------------------------------
		// 				} else
		// 				{
		// 						ROS_WARN("Invalid/missing operation mode");
		// 				}

		// 				motorController.getPosition(motorIDs, positions);
		// 				std_msgs::Int64MultiArray motorPos;
		// 				for (int i = 0; i < positions.size(); ++i) motorPos.data[i] = positions[i];
		// 				//motorPosPrev.data[i] = motorPos.data[i];
		// 				pub.publish(motorPos);
		// 		}

		// 		rate.sleep();
		// 		ros::spinOnce();
		// }

		// /**std_msgs::Int64MultiArray motorPosPrev;

		//    for (int i = 0; i < motorIDs.size(); ++i) motorPosPrev.data.push_back(0);
		//    for (int i = 0; i < motorIDs.size(); ++i) motorPosPrev.data.push_back(1);

		//    while(ros::ok())     // && iteration < 500)
		//    {
		//     // I suspect these changes may need to be made on a per motor basis to get desired behavior
		//     bool moving = false;
		//     bool fault = false;
		//     for (int i = 0; i < vels.size(); ++i) if (vels[i] != 0) moving = true;
		//     for (int i = 0; i < motorIDs.size(); ++i) if (motorPosPrev.data[i] == motorPos.data[i]) fault = true;
		//     if (!( moving && !fault))
		//     {

		//         ROS_INFO("FAULT TRIGGERED");
		//         prepareCheck = motorController.enableMotors(motorIDs);

		//     }


		//     //++iteration;
		//    }		 */

		// // STOP AND CLOSE MOTORS---------------------------------------------------------------------
		// std::vector<long> stopVels;
		// for (int i = 0; motorIDs.size(); ++i) stopVels.push_back(0);
		// motorController.goToVel(motorIDs,stopVels);
		// motorController.closeDevices();

		rclcpp::shutdown();
		return 0;

}
