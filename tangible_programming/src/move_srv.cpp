#include <iostream>

#include "ros/ros.h"

#include "tangible_msgs/GetMovements.h"

bool move_callback(tangible_msgs::GetMovements::Request& req, tangible_msgs::GetMovements::Response& res)
{
	res.status = true;
	// int input; // to simulate actual service
	// ROS_INFO("arm moved? 1: YES");
	// std::cin >> input;
	// if(input == 1)
	// 	res.success = true;
	// else
	// 	res.success = false;
	return true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "arm_movement_server");

	ros::NodeHandle n;
	ros::ServiceServer srv = n.advertiseService("move_arm", move_callback);
	ROS_INFO("fake service to move the arm");
	ros::spin();

	return 0;
}