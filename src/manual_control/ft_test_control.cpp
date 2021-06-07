#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <epos_ros/motor_command.h>

double maxV;
ros::Publisher motor_cmd_pub;
std::vector<short int> motorIDs;
std::string operationMode = "velocity";

void cmdVelCallBack(const geometry_msgs::Twist::ConstPtr& msg)
{

  int drive = msg->linear.x*maxV;
  int turn = msg->angular.z*maxV;

	epos_ros::motor_command cmd_msg;
	cmd_msg.motorIDs = motorIDs;
	cmd_msg.numberItems = motorIDs.size();
              // 1      2     4      5     8     9      16     31
  cmd_msg.command = {drive, turn, drive, turn, turn, drive, drive, turn};
	motor_cmd_pub.publish(cmd_msg);
}

int main(int argc, char** argv)
{

	ros::init(argc, argv, "subscriber");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("/cmd_vel",10,&cmdVelCallBack);

	motor_cmd_pub = nh.advertise<epos_ros::motor_command>("/velocities", 1000);

  /**std::vector<double> tempIDs;
	nh.param("motors_ids/list", tempIDs);
  for (auto& id: tempIDs) motorIDs.push_back(id);
	nh.param("user_limit/ang_vel_rpm", maxV);
  */
  motorIDs.push_back(1);
  motorIDs.push_back(2);
  motorIDs.push_back(4);
  motorIDs.push_back(5);
  motorIDs.push_back(8);
  motorIDs.push_back(9);
  motorIDs.push_back(16);
  motorIDs.push_back(31);
  maxV = 3000;

  ROS_INFO("maxV: %f",maxV);

	ros::spin();
}
