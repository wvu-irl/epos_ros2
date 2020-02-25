/*
   Maxon Motor Controller test node epos_test_node
   epos_test_node.cpp
   Purpose: Run epos commands and test behavior

   @author Jared Beard
   @version 1.0 1/18/19
 */
#include <ros/ros.h>
#include <ros/console.h>
#include <epos/epos_cmd.h>
#include <epos/wheel_drive.h>
#include <std_msgs/Int64MultiArray.h>
//#include <geometry_msgs/Twist.h>

#include <stdio.h>
#include <cmath>

#define COUNTS_PER_REV 128
#define COUNTS_PER_REV_SHAFT (128.0*2000/6.5)
//6.5 rev of output ~= to 2k rev motor

std::vector<int> motorIDs;
std::vector<long> vels;


void motorCommandCallback(const epos::wheel_drive &msg)
{
//std::cout << msg.numberItems;
		for (int i = 0; i < msg.numberItems; ++i)
		{

				motorIDs[i] = msg.motorIDs[i];
				vels[i] = msg.command[i];
		}
}


int main(int argc, char** argv)
{
		ros::init(argc,argv,"subscriber");
		ros::NodeHandle nh;
		ros::Subscriber sub = nh.subscribe("/velocities", 10, motorCommandCallback);
		ros::NodeHandle n;
		ros::Publisher pub = n.advertise<std_msgs::Int64MultiArray>("/motor_pos", 1000);
        ros::Publisher pubCurrent = n.advertise<std_msgs::Int64MultiArray>("/motor_currents", 1000);

		motorIDs.push_back(1);
		motorIDs.push_back(2);
		motorIDs.push_back(3);
		motorIDs.push_back(4);
		//std::vector<unsigned short> motorIDshort;
		//motorIDshort.push_back(1);
		//motorIDshort.push_back(2);
		//motorIDs.push_back(3);
		//motorIDs.push_back(4);

		vels.push_back(0);
		vels.push_back(0);
		vels.push_back(0);
		vels.push_back(0);
		std_msgs::Int64MultiArray motorPosPrev;
		std_msgs::Int64MultiArray motorPos;
		std_msgs::Int64MultiArray motorCurrents;
		for (int i = 0; i < motorIDs.size(); ++i)	motorPos.data.push_back(0);
		for (int i = 0; i < motorIDs.size(); ++i)	motorPosPrev.data.push_back(1);
		for (int i = 0; i < motorIDs.size(); ++i)	motorCurrents.data.push_back(0); 	
		std::vector<long> stopVels = vels;
		int baudrate = 1000000;
		//std::cout << motorIDs[0] << "__" << motorIDs[1] << std::endl;

		std::vector<int> positions;
        std::vector<short> currents; 

		epos_cmd motorController(motorIDs, baudrate);

		if (!motorController.openDevices())
		{
				ROS_FATAL("Motor not opened");
				return 1;
		}
		
		try{    
		        
				float iteration = 0;
				motorController.setMode(motorIDs, epos_cmd::OMD_PROFILE_VELOCITY_MODE);
				
				int prepareCheck = motorController.prepareMotors(motorIDs);

				ros::Rate rate(60);
				while(ros::ok()) // && iteration < 500)
				{
						// I suspect these changes may need to be made on a per motor basis to get desired behavior
						bool moving = false;
						bool fault = false;

						for (int i = 0; i < vels.size(); ++i)if (vels[i] != 0)   moving = true;


						prepareCheck = motorController.prepareMotors(motorIDs);

						motorController.goToVel(motorIDs, vels);
						
						positions.clear();
						currents.clear(); 						
						motorController.getPosition(motorIDs, positions);

                        //std::cout << positions.size() << std::endl;
						for (int i = 0; i < positions.size(); ++i)
						{
								motorPosPrev.data[i] = motorPos.data[i];
								motorPos.data[i] = positions[i];
								//std::cout << "Umm..." << std::endl;
								//std::cout << motorPos.data[i] << std::endl;
						}
						
                        motorController.getCurrent(motorIDs, currents); 
                        for (int i = 0; i < currents.size(); ++i)
						{
								motorCurrents.data[i] = currents[i];
								//std::cout << "Umm..." << std::endl;
								//std::cout << motorPos.data[i] << std::endl;
						}
						
						pub.publish(motorPos);
						pubCurrent.publish(motorCurrents); 

						//++iteration;
						rate.sleep();
						ros::spinOnce();

				}

				motorController.goToVel(motorIDs,stopVels);
				motorController.closeDevices();
				return 0;
		} catch (const std::exception& e)
		{
				motorController.closeDevices();
				return 1;
		}
}
